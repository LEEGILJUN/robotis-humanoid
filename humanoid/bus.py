"""Bus backends: a real DYNAMIXEL bus (via U2D2 / USB2Dynamixel) and a simulator.

Both expose the same small interface so every tool can run without hardware:
    ping(id) -> model number | None
    read(id, addr, length) -> (data, error_byte)
    write(id, addr, length, value) -> error_byte
    sync_write(addr, length, {id: value})
    sleep(seconds)
"""
from __future__ import annotations

import glob
import random
import time

from .control_table import ADDR, MODELS, word


class CommError(RuntimeError):
    pass


def find_ports() -> list[str]:
    patterns = ["/dev/cu.usbserial-*", "/dev/cu.usbmodem*", "/dev/ttyUSB*", "/dev/ttyACM*"]
    ports: list[str] = []
    for pattern in patterns:
        ports.extend(sorted(glob.glob(pattern)))
    return ports


class DxlBus:
    def __init__(self, port: str, baudrate: int, protocol: float = 1.0, retries: int = 2):
        import dynamixel_sdk as sdk

        self._sdk = sdk
        self.port_name = port
        self.retries = retries
        self.port = sdk.PortHandler(port)
        if not self.port.openPort():
            raise CommError(f"포트를 열 수 없습니다: {port}")
        self.ph = sdk.PacketHandler(protocol)
        self.set_baudrate(baudrate)

    def set_baudrate(self, baudrate: int) -> None:
        if not self.port.setBaudRate(baudrate):
            raise CommError(f"보드레이트 설정 실패: {baudrate}")
        self.baudrate = baudrate

    def ping(self, dxl_id: int) -> int | None:
        model, result, _ = self.ph.ping(self.port, dxl_id)
        return model if result == self._sdk.COMM_SUCCESS else None

    def read(self, dxl_id: int, addr: int, length: int) -> tuple[list[int], int]:
        result = None
        for _ in range(self.retries + 1):
            data, result, err = self.ph.readTxRx(self.port, dxl_id, addr, length)
            if result == self._sdk.COMM_SUCCESS:
                return list(data), err
        raise CommError(f"ID {dxl_id} 읽기 실패: {self.ph.getTxRxResult(result)}")

    def write(self, dxl_id: int, addr: int, length: int, value: int) -> int:
        fn = self.ph.write1ByteTxRx if length == 1 else self.ph.write2ByteTxRx
        result = None
        for _ in range(self.retries + 1):
            result, err = fn(self.port, dxl_id, addr, value)
            if result == self._sdk.COMM_SUCCESS:
                return err
        raise CommError(f"ID {dxl_id} 쓰기 실패: {self.ph.getTxRxResult(result)}")

    def sync_write(self, addr: int, length: int, values: dict[int, int]) -> None:
        group = self._sdk.GroupSyncWrite(self.port, self.ph, addr, length)
        for dxl_id, value in values.items():
            group.addParam(dxl_id, [value & 0xFF, (value >> 8) & 0xFF][:length])
        result = group.txPacket()
        if result != self._sdk.COMM_SUCCESS:
            raise CommError(f"sync write 실패: {self.ph.getTxRxResult(result)}")

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)

    def now(self) -> float:
        return time.monotonic()

    def close(self) -> None:
        self.port.closePort()


# --------------------------------------------------------------------------- simulator

SIM_FAULTS = ("missing", "stuck", "hot", "jitter", "shutdown", "wheel")


class SimMotor:
    """Very small behavioural model of an AX/MX servo, good enough to exercise the tools."""

    def __init__(self, dxl_id: int, model_number: int, fault: str | None, rng: random.Random):
        self.id = dxl_id
        self.model = MODELS[model_number]
        self.fault = fault
        self.rng = rng
        self.mem = bytearray(74)
        self.error = 0
        self.stall_time = 0.0
        res = self.model.resolution
        scale = res // 1024
        self._set("model_number", model_number)
        self._set("firmware", 24 if self.model.family == "AX" else 39)
        self._set("id", dxl_id)
        self._set("baud_rate", 1)
        self._set("cw_angle_limit", 0)
        self._set("ccw_angle_limit", 0 if fault == "wheel" else res - 1)
        self._set("temperature_limit", 70)
        self._set("min_voltage", 60)
        self._set("max_voltage", 140)
        self._set("max_torque", 1023)
        self._set("status_return_level", 2)
        self._set("alarm_led", 36)
        self._set("shutdown", 36)
        self.pos = float(self.model.center + rng.randint(-60, 60) * scale)
        self._set("goal_position", round(self.pos))
        self._set("torque_limit", 0 if fault == "shutdown" else 1023)
        self._set("present_voltage", 121)
        self.temperature = 72 if fault == "hot" else 31 + rng.randint(0, 4)
        if fault == "hot":
            self.error |= 1 << 2
        # Wheel-mode mechanism model: continuous shaft angle relative to the start,
        # optional mechanical stops (lo, hi) in degrees, absolute angle of the start.
        self.cont = 0.0
        self.stops: tuple[float, float] | None = None
        self.base_deg = self.pos * self.model.deg_per_tick
        self.spun_deg = 0.0
        self._update_present(0, False)

    def _set(self, name: str, value: int) -> None:
        addr, size = ADDR[name]
        self.mem[addr] = value & 0xFF
        if size == 2:
            self.mem[addr + 1] = (value >> 8) & 0xFF

    def _get(self, name: str) -> int:
        addr, size = ADDR[name]
        return word(self.mem, addr) if size == 2 else self.mem[addr]

    def _update_present(self, load: int, moving: bool, speed_rpm: float = 0.0) -> None:
        res = self.model.resolution
        spd = min(round(abs(speed_rpm) / self.model.speed_unit_rpm), 1023)
        self._set("present_speed", spd | (0x400 if speed_rpm < 0 else 0))
        noise = self.rng.randint(-6, 6) * (res // 1024) if self.fault == "jitter" else 0
        self._set("present_position", min(max(round(self.pos) + noise, 0), res - 1))
        mag = min(abs(load), 1023)
        self._set("present_load", mag | (0x400 if load < 0 else 0))
        self._set("present_temperature", self.temperature)
        self._set("moving", int(moving))
        if self.model.has_current:
            self._set("present_current", 2048 + round(load * 0.9))

    def step(self, dt: float) -> None:
        torque_on = self._get("torque_enable") == 1
        torque_limit = self._get("torque_limit")
        goal = self._get("goal_position")
        delta = goal - self.pos
        load = 0
        moving = False
        wheel_mode = self._get("cw_angle_limit") == 0 and self._get("ccw_angle_limit") == 0
        speed_actual = 0.0
        if torque_on and torque_limit > 0 and wheel_mode:
            # Wheel mode: Moving Speed is a rotation speed, goal position is ignored.
            # Moving Speed is an output ratio here (0-1023 = 0-100 %), not a closed-loop speed.
            raw = self._get("moving_speed")
            output = min(raw & 0x3FF, torque_limit) / 1023.0
            friction = 0.06  # below ~6 % output the mechanism does not move
            frac = max(0.0, output - friction) / (1 - friction)
            rpm = self.model.max_rpm * frac * (-1 if raw & 0x400 else 1)
            step = rpm * 6.0 * dt
            new = self.cont + step
            sign = 1 if step > 0 else -1
            blocked = False
            if self.stops is not None:
                lo, hi = self.stops
                if (step > 0 and new > hi) or (step < 0 and new < lo):
                    new = min(max(new, lo), hi)
                    blocked = True
            self.spun_deg += abs(new - self.cont)
            self.cont = new
            moving = rpm != 0 and not blocked
            speed_actual = rpm if moving else 0.0
            if blocked or (output > 0 and not moving):
                load = sign * round(output * 1023)
            elif moving:
                load = sign * round(friction * 1023 + 20)
            absolute = (self.base_deg + self.cont) % 360.0
            if absolute < self.model.range_deg:
                self.pos = absolute / self.model.deg_per_tick
            else:  # potentiometer dead zone: the reading is meaningless
                self.pos = float(self.rng.choice([0, self.model.resolution - 1]))
        elif torque_on and torque_limit > 0:
            if self.fault == "stuck":
                moving = abs(delta) > 2
                if moving:
                    self.stall_time += dt
                    load = int(min(torque_limit, 1023) * (1 if delta > 0 else -1))
                    if self.stall_time > 0.6:  # overload shutdown, like the real servo
                        self.error |= 1 << 5
                        self._set("torque_enable", 0)
                        self._set("torque_limit", 0)
            else:
                speed_raw = self._get("moving_speed") & 0x3FF
                rpm = speed_raw * self.model.speed_unit_rpm if speed_raw else self.model.max_rpm
                rpm = min(rpm, self.model.max_rpm)
                max_step = rpm * 6.0 * dt / self.model.deg_per_tick
                move = max(-max_step, min(max_step, delta))
                self.pos += move
                moving = abs(goal - self.pos) > 1
                sign = 1 if delta >= 0 else -1
                load = int(sign * (150 + (250 if moving else 0)))
                load = max(-torque_limit, min(torque_limit, load))
                speed_actual = (rpm if delta >= 0 else -rpm) if moving else 0.0
        self._update_present(load, moving, speed_actual)


class SimBus:
    def __init__(self, motors: dict[int, int], faults: dict[int, str] | None = None, seed: int = 0):
        rng = random.Random(seed)
        faults = faults or {}
        self.motors = {i: SimMotor(i, num, faults.get(i), rng) for i, num in motors.items()}
        self.time = 0.0
        self.port_name = "sim"
        self.baudrate = 1_000_000

    def set_baudrate(self, baudrate: int) -> None:
        self.baudrate = baudrate

    def _advance(self, dt: float) -> None:
        for m in self.motors.values():
            m.step(dt)
        self.time += dt

    def _motor(self, dxl_id: int) -> SimMotor:
        m = self.motors.get(dxl_id)
        if m is None or m.fault == "missing" or self.baudrate != 1_000_000:
            raise CommError(f"ID {dxl_id} 응답 없음 (sim)")
        return m

    def ping(self, dxl_id: int) -> int | None:
        self._advance(0.001)
        try:
            return self._motor(dxl_id).model.number
        except CommError:
            return None

    def read(self, dxl_id: int, addr: int, length: int) -> tuple[list[int], int]:
        self._advance(0.001)
        m = self._motor(dxl_id)
        return list(m.mem[addr:addr + length]), m.error

    def write(self, dxl_id: int, addr: int, length: int, value: int) -> int:
        self._advance(0.001)
        m = self._motor(dxl_id)
        m.mem[addr] = value & 0xFF
        if length == 2:
            m.mem[addr + 1] = (value >> 8) & 0xFF
        if addr == ADDR["id"][0]:  # the motor now answers to its new ID
            self.motors[value] = self.motors.pop(dxl_id)
            m.id = value
        return m.error

    def sync_write(self, addr: int, length: int, values: dict[int, int]) -> None:
        self._advance(0.001)
        for dxl_id, value in values.items():
            m = self.motors.get(dxl_id)
            if m is None or m.fault == "missing":
                continue
            m.mem[addr] = value & 0xFF
            if length == 2:
                m.mem[addr + 1] = (value >> 8) & 0xFF

    def sleep(self, seconds: float) -> None:
        steps = max(1, round(seconds / 0.01))
        for _ in range(steps):
            self._advance(seconds / steps)

    def now(self) -> float:
        return self.time

    def close(self) -> None:
        pass
