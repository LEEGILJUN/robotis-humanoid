"""DYNAMIXEL Protocol 1.0 control table for the AX and MX series.

Only the registers used by the diagnostic tools are listed. Addresses follow
the ROBOTIS e-Manual for AX-12A / AX-18A and MX-28 / MX-64 / MX-106 (Protocol 1.0).
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Model:
    number: int
    name: str
    family: str            # "AX" or "MX"
    resolution: int        # number of position ticks
    range_deg: float       # angle covered by the ticks
    max_rpm: float         # no-load speed at 12 V
    stall_torque_nm: float  # stall torque at 12 V
    speed_unit_rpm: float  # rpm per Moving Speed unit
    has_current: bool = False

    @property
    def center(self) -> int:
        return self.resolution // 2

    @property
    def deg_per_tick(self) -> float:
        return self.range_deg / self.resolution

    def ticks_to_deg(self, ticks: float) -> float:
        """Angle relative to the center position."""
        return (ticks - self.center) * self.deg_per_tick

    def deg_to_ticks(self, deg: float) -> int:
        return round(deg / self.deg_per_tick) + self.center


MODELS: dict[int, Model] = {
    12: Model(12, "AX-12A", "AX", 1024, 300.0, 59, 1.5, 0.111),
    18: Model(18, "AX-18A", "AX", 1024, 300.0, 97, 1.8, 0.111),
    300: Model(300, "AX-12W", "AX", 1024, 300.0, 470, 0.2, 0.111),
    29: Model(29, "MX-28", "MX", 4096, 360.0, 55, 2.5, 0.114),
    310: Model(310, "MX-64", "MX", 4096, 360.0, 63, 6.0, 0.114, has_current=True),
    320: Model(320, "MX-106", "MX", 4096, 360.0, 45, 8.4, 0.114, has_current=True),
}
MODELS_BY_NAME: dict[str, Model] = {m.name: m for m in MODELS.values()}

# name -> (address, size in bytes)
ADDR: dict[str, tuple[int, int]] = {
    # EEPROM
    "model_number": (0, 2),
    "firmware": (2, 1),
    "id": (3, 1),
    "baud_rate": (4, 1),
    "return_delay": (5, 1),
    "cw_angle_limit": (6, 2),
    "ccw_angle_limit": (8, 2),
    "temperature_limit": (11, 1),
    "min_voltage": (12, 1),
    "max_voltage": (13, 1),
    "max_torque": (14, 2),
    "status_return_level": (16, 1),
    "alarm_led": (17, 1),
    "shutdown": (18, 1),
    # RAM
    "torque_enable": (24, 1),
    "led": (25, 1),
    "goal_position": (30, 2),
    "moving_speed": (32, 2),
    "torque_limit": (34, 2),
    "present_position": (36, 2),
    "present_speed": (38, 2),
    "present_load": (40, 2),
    "present_voltage": (42, 1),
    "present_temperature": (43, 1),
    "moving": (46, 1),
    "punch": (48, 2),
    # MX-64 / MX-106 only
    "present_current": (68, 2),
}

# Contiguous blocks read in one transaction.
EEPROM_START, EEPROM_LEN = 0, 19   # 0..18
RAM_START, RAM_LEN = 24, 26        # 24..49

# Status packet error byte (Protocol 1.0)
ERROR_BITS = {
    0: "input_voltage",
    1: "angle_limit",
    2: "overheating",
    3: "range",
    4: "checksum",
    5: "overload",
    6: "instruction",
}

# Baud Rate register value -> bps is 2_000_000 / (value + 1). Common settings:
COMMON_BAUDRATES = [1_000_000, 57_600, 115_200, 500_000, 400_000, 250_000, 200_000, 19_200, 9_600]


def word(data: list[int] | bytes, offset: int) -> int:
    return data[offset] | (data[offset + 1] << 8)


def decode_signed10(raw: int) -> int:
    """Present Load / Present Speed: bits 0-9 magnitude, bit 10 direction (1 = CW)."""
    mag = raw & 0x3FF
    return -mag if raw & 0x400 else mag


def decode_errors(err: int) -> list[str]:
    return [name for bit, name in ERROR_BITS.items() if err & (1 << bit)]
