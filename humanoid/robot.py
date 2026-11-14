"""Robot configuration and shared helpers for the command-line tools."""
from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import yaml

from .bus import SIM_FAULTS, CommError, DxlBus, SimBus, find_ports
from .control_table import MODELS_BY_NAME, Model

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = ROOT / "config" / "robot.yaml"
LOG_DIR = Path(os.environ.get("HUMANOID_LOG_DIR", ROOT / "logs"))
POSE_DIR = Path(os.environ.get("HUMANOID_POSE_DIR", ROOT / "poses"))


@dataclass
class Joint:
    id: int
    name: str
    model: str

    @property
    def model_info(self) -> Model | None:
        return MODELS_BY_NAME.get(self.model)


@dataclass
class RobotConfig:
    protocol: float
    baudrate: int
    joints: dict[int, Joint]
    test: dict

    def name_of(self, dxl_id: int) -> str:
        j = self.joints.get(dxl_id)
        return j.name if j else "?"


def load_config(path: str | Path = DEFAULT_CONFIG) -> RobotConfig:
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    joints = {int(i): Joint(int(i), j["name"], j["model"]) for i, j in (raw.get("joints") or {}).items()}
    bus = raw.get("bus", {})
    return RobotConfig(
        protocol=float(bus.get("protocol", 1.0)),
        baudrate=int(bus.get("baudrate", 1_000_000)),
        joints=joints,
        test=raw.get("test", {}),
    )


# ------------------------------------------------------------------ CLI helpers

# Faults injected by `--port sim --sim-faults`, to see what a bad motor looks like in the reports.
DEMO_FAULTS = {11: "missing", 14: "stuck", 16: "hot", 5: "jitter", 3: "shutdown"}
# Extra simulated motors that are not in the config.
SIM_EXTRA_MOTORS: dict[int, int] = {}
# Mirrors the real robot: both AX-18A grippers are configured in wheel mode.
SIM_DEFAULT_FAULTS = {19: "wheel", 20: "wheel"}
SIM_GRIPPER_STOPS = {19: (0.0, 170.0), 20: (-170.0, 0.0)}


def add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--config", default=str(DEFAULT_CONFIG), help="robot config YAML")
    p.add_argument("--port", help="serial port (e.g. /dev/cu.usbserial-XXXX). 'sim' = simulator")
    p.add_argument("--baud", type=int, help="override baudrate")
    p.add_argument("--sim-faults", action="store_true", help="simulator: inject demo faults")
    p.add_argument("--yes", "-y", action="store_true", help="skip confirmation prompts")


def open_bus(args, cfg: RobotConfig):
    baud = args.baud or cfg.baudrate
    if args.port == "sim":
        motors = {j.id: j.model_info.number for j in cfg.joints.values() if j.model_info}
        motors.update({k: v for k, v in SIM_EXTRA_MOTORS.items() if k not in motors})
        faults = dict(SIM_DEFAULT_FAULTS)
        if args.sim_faults:
            faults.update(DEMO_FAULTS)
        bus = SimBus(motors, faults)
        # Grippers: closed at the start, mirrored opening direction, travel crosses the dead zone.
        for gid, stops in SIM_GRIPPER_STOPS.items():
            if gid in bus.motors:
                bus.motors[gid].stops = stops
                bus.motors[gid].base_deg = 250.0
        bus.set_baudrate(baud)
        return bus
    port = args.port
    if not port:
        ports = find_ports()
        if not ports:
            sys.exit("U2D2(USB 시리얼 포트)를 찾지 못했습니다. 연결을 확인하거나, 시뮬레이터는 --port sim 으로 실행하세요.")
        if len(ports) > 1:
            sys.exit("시리얼 포트가 여러 개입니다. --port 로 지정하세요:\n  " + "\n  ".join(ports))
        port = ports[0]
    try:
        return DxlBus(port, baud, cfg.protocol)
    except CommError as e:
        sys.exit(str(e))


def confirm(message: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    answer = input(f"{message} [Enter=진행 / s=건너뛰기 / q=종료] ").strip().lower()
    if answer == "q":
        raise KeyboardInterrupt
    return answer != "s"


def save_log(kind: str, data) -> Path:
    LOG_DIR.mkdir(exist_ok=True)
    path = LOG_DIR / f"{kind}_{datetime.now():%Y%m%d_%H%M%S}.json"
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def safe_torque_off(bus, ids) -> None:
    for i in ids:
        try:
            bus.write(i, 24, 1, 0)
        except CommError:
            pass
