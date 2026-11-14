"""Reading and judging the state of a single servo."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

from .bus import CommError
from .control_table import (
    ADDR, EEPROM_LEN, EEPROM_START, MODELS, RAM_LEN, RAM_START, Model,
    decode_errors, decode_signed10, word,
)


@dataclass
class MotorStatus:
    id: int
    model_number: int
    model_name: str
    firmware: int
    cw_angle_limit: int
    ccw_angle_limit: int
    temperature_limit: int
    min_voltage: float
    max_voltage: float
    max_torque: int
    torque_enable: int
    goal_position: int
    moving_speed: int
    torque_limit: int
    position: int
    position_deg: float
    load_pct: float
    speed_raw: int
    voltage: float
    temperature: int
    moving: bool
    error: int
    errors: list[str] = field(default_factory=list)
    current_ma: float | None = None

    def to_dict(self) -> dict:
        return asdict(self)


def model_of(number: int) -> Model | None:
    return MODELS.get(number)


def is_wheel_mode(bus, dxl_id: int) -> bool:
    """Wheel mode (both angle limits 0): Moving Speed spins the motor endlessly, so
    position-based tools must never enable torque on it."""
    data, _ = bus.read(dxl_id, ADDR["cw_angle_limit"][0], 4)
    return word(data, 0) == 0 and word(data, 2) == 0


def _ram_fields(ram: list[int], model: Model | None) -> dict:
    off = RAM_START
    pos = word(ram, ADDR["present_position"][0] - off)
    return {
        "torque_enable": ram[ADDR["torque_enable"][0] - off],
        "goal_position": word(ram, ADDR["goal_position"][0] - off),
        "moving_speed": word(ram, ADDR["moving_speed"][0] - off),
        "torque_limit": word(ram, ADDR["torque_limit"][0] - off),
        "position": pos,
        "position_deg": round(model.ticks_to_deg(pos), 2) if model else 0.0,
        "load_pct": round(decode_signed10(word(ram, ADDR["present_load"][0] - off)) / 10.23, 1),
        "speed_raw": decode_signed10(word(ram, ADDR["present_speed"][0] - off)),
        "voltage": ram[ADDR["present_voltage"][0] - off] / 10,
        "temperature": ram[ADDR["present_temperature"][0] - off],
        "moving": bool(ram[ADDR["moving"][0] - off]),
    }


def read_status(bus, dxl_id: int) -> MotorStatus:
    eeprom, err1 = bus.read(dxl_id, EEPROM_START, EEPROM_LEN)
    ram, err2 = bus.read(dxl_id, RAM_START, RAM_LEN)
    number = word(eeprom, ADDR["model_number"][0])
    model = model_of(number)
    err = err1 | err2
    current = None
    if model and model.has_current:
        try:
            raw, _ = bus.read(dxl_id, *ADDR["present_current"])
            current = round(4.5 * (word(raw, 0) - 2048), 1)
        except CommError:
            current = None
    return MotorStatus(
        id=dxl_id,
        model_number=number,
        model_name=model.name if model else f"unknown({number})",
        firmware=eeprom[ADDR["firmware"][0]],
        cw_angle_limit=word(eeprom, ADDR["cw_angle_limit"][0]),
        ccw_angle_limit=word(eeprom, ADDR["ccw_angle_limit"][0]),
        temperature_limit=eeprom[ADDR["temperature_limit"][0]],
        min_voltage=eeprom[ADDR["min_voltage"][0]] / 10,
        max_voltage=eeprom[ADDR["max_voltage"][0]] / 10,
        max_torque=word(eeprom, ADDR["max_torque"][0]),
        error=err,
        errors=decode_errors(err),
        current_ma=current,
        **_ram_fields(ram, model),
    )


def read_live(bus, dxl_id: int, model: Model | None) -> dict:
    """Fast RAM-only read for control / monitoring loops."""
    ram, err = bus.read(dxl_id, RAM_START, RAM_LEN)
    fields = _ram_fields(ram, model)
    fields["error"] = err
    fields["errors"] = decode_errors(err)
    return fields


# Severity levels used in reports
FAIL, WARN, INFO = "FAIL", "WARN", "INFO"


def health_issues(st: MotorStatus, expected_model: str | None, limits: dict) -> list[tuple[str, str]]:
    issues: list[tuple[str, str]] = []
    if expected_model and st.model_name != expected_model:
        issues.append((WARN, f"모델 불일치: 설정={expected_model}, 실제={st.model_name}"))
    if st.errors:
        issues.append((FAIL, f"에러 비트: {', '.join(st.errors)}"))
    if st.torque_limit == 0:
        issues.append((WARN, "Torque Limit=0 (이전에 과부하·과열로 차단된 상태, 시험 시 복구함)"))
    if st.cw_angle_limit == 0 and st.ccw_angle_limit == 0:
        issues.append((WARN, "Wheel 모드로 설정됨 (각도 제한이 모두 0)"))
    if not (st.min_voltage <= st.voltage <= st.max_voltage):
        issues.append((FAIL, f"전압 범위 이탈: {st.voltage:.1f}V"))
    elif st.voltage < limits.get("min_voltage_v", 10.0):
        issues.append((WARN, f"전압 낮음: {st.voltage:.1f}V (전원 용량 확인)"))
    if st.temperature >= limits.get("max_temp_c", 60):
        issues.append((WARN, f"온도 높음: {st.temperature}°C"))
    if st.max_torque < 512:
        issues.append((INFO, f"Max Torque가 낮게 설정됨: {st.max_torque}/1023"))
    return issues


def verdict(issues: list[tuple[str, str]]) -> str:
    levels = {lvl for lvl, _ in issues}
    if FAIL in levels:
        return FAIL
    if WARN in levels:
        return WARN
    return "PASS"
