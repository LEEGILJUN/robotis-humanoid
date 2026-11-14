"""Whole-robot test: record poses by hand, play them back and watch every joint's load.

    python -m tools.pose record sit          # torque off, shape the robot by hand, press Enter
    python -m tools.pose play sit --duration 2 --hold 10
    python -m tools.pose monitor             # live table of every joint (Ctrl+C to stop)
    python -m tools.pose release             # torque off everything
    python -m tools.pose list
"""
from __future__ import annotations

import argparse
from datetime import datetime

import yaml

from humanoid.bus import CommError
from humanoid.control_table import ADDR, MODELS
from humanoid.robot import (
    POSE_DIR, add_common_args, confirm, load_config, open_bus, safe_torque_off, save_log,
)
from humanoid.status import is_wheel_mode, read_live


def present_motors(bus, ids, include_wheel: bool = False) -> dict[int, object]:
    """id -> Model for every responding motor. Wheel-mode motors are left out unless
    include_wheel, because enabling torque with a Moving Speed spins them endlessly."""
    out, wheel = {}, []
    for i in ids:
        number = bus.ping(i)
        if number is None or number not in MODELS:
            continue
        if not include_wheel and is_wheel_mode(bus, i):
            wheel.append(i)
            continue
        out[i] = MODELS[number]
    if wheel:
        print(f"Wheel 모드 모터는 제외: {wheel}")
    return out


def read_positions(bus, motors) -> dict[int, int]:
    return {i: read_live(bus, i, m)["position"] for i, m in motors.items()}


def cmd_record(bus, cfg, args) -> dict:
    motors = present_motors(bus, range(0, 31))
    print(f"응답한 모터 {len(motors)}개: {sorted(motors)}")
    if args.keep_torque:
        print("토크를 건드리지 않고 현재 자세를 기록합니다.")
    else:
        if not confirm("모든 모터의 토크를 해제합니다. 로봇을 손으로 잡으세요.", args.yes):
            return {}
        safe_torque_off(bus, motors)
        if not args.yes:
            input("원하는 자세를 손으로 만든 뒤 Enter를 누르세요... ")
    pos = read_positions(bus, motors)
    pose = {
        "name": args.name,
        "recorded_at": datetime.now().isoformat(timespec="seconds"),
        "joints": {i: {"name": cfg.name_of(i), "model": motors[i].name, "position": p,
                       "deg": round(motors[i].ticks_to_deg(p), 2)} for i, p in sorted(pos.items())},
    }
    POSE_DIR.mkdir(exist_ok=True)
    path = POSE_DIR / f"{args.name}.yaml"
    path.write_text(yaml.safe_dump(pose, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"저장: {path}")
    return pose


def sample_all(bus, motors) -> dict[int, dict]:
    out = {}
    for i, m in motors.items():
        try:
            out[i] = read_live(bus, i, m)
        except CommError:
            out[i] = None
    return out


def cmd_play(bus, cfg, args) -> dict:
    path = POSE_DIR / f"{args.name}.yaml"
    pose = yaml.safe_load(path.read_text(encoding="utf-8"))
    targets = {int(i): j["position"] for i, j in pose["joints"].items()}
    motors = present_motors(bus, targets)
    skipped = sorted(set(targets) - set(motors))
    if skipped:
        print(f"응답 없는 관절은 건너뜀: {skipped}")
    targets = {i: targets[i] for i in motors}

    current = read_positions(bus, motors)
    speeds = {}
    for i, m in motors.items():
        deg = abs(targets[i] - current[i]) * m.deg_per_tick
        rpm = deg / max(args.duration, 0.1) / 6.0
        speeds[i] = max(1, min(1023, round(rpm / m.speed_unit_rpm)))
    torque_limit = max(1, round(1023 * args.torque_pct / 100))

    if not confirm(f"'{args.name}' 자세로 {args.duration}초 동안 이동합니다 (토크 {args.torque_pct}%). "
                   "로봇을 잡아 주세요.", args.yes):
        return {}
    try:
        bus.sync_write(*ADDR["goal_position"], current)  # avoid a jump when torque turns on
        bus.sync_write(*ADDR["torque_limit"], {i: torque_limit for i in motors})
        bus.sync_write(*ADDR["moving_speed"], speeds)
        bus.sync_write(*ADDR["torque_enable"], {i: 1 for i in motors})
        bus.sync_write(*ADDR["goal_position"], targets)
        bus.sleep(args.duration + 0.3)

        stats = {i: {"peak_load_pct": 0.0, "max_temp": 0, "max_error_deg": 0.0, "errors": set()} for i in motors}
        t0 = bus.now()
        print(f"{args.hold}초 동안 유지하며 부하를 기록합니다...")
        while bus.now() - t0 < args.hold:
            for i, live in sample_all(bus, motors).items():
                s = stats[i]
                if live is None:
                    s["errors"].add("comm")
                    continue
                s["peak_load_pct"] = max(s["peak_load_pct"], abs(live["load_pct"]))
                s["max_temp"] = max(s["max_temp"], live["temperature"])
                err = abs(live["position"] - targets[i]) * motors[i].deg_per_tick
                s["max_error_deg"] = max(s["max_error_deg"], round(err, 2))
                s["errors"] |= set(live["errors"])
            bus.sleep(0.2)
    except KeyboardInterrupt:
        print("\n중단했습니다.")
        if confirm("토크를 해제할까요? 로봇을 잡은 뒤 진행하세요.", False):
            safe_torque_off(bus, motors)
        raise

    print(f"\n{'ID':>3} {'NAME':<17} {'LOAD%':>6} {'TEMP':>5} {'ERR(deg)':>8}  NOTE")
    for i, s in sorted(stats.items(), key=lambda kv: -kv[1]["peak_load_pct"]):
        notes = []
        if s["peak_load_pct"] > 70:
            notes.append("부하 높음")
        if s["max_error_deg"] > 3:
            notes.append("자세 유지 오차 큼")
        if s["errors"]:
            notes.append(f"에러 {sorted(s['errors'])}")
        print(f"{i:>3} {cfg.name_of(i):<17} {s['peak_load_pct']:>6.0f} {s['max_temp']:>5} "
              f"{s['max_error_deg']:>8.1f}  {', '.join(notes)}")
        s["errors"] = sorted(s["errors"])

    if args.release or (not args.yes and confirm("토크를 해제할까요? 로봇이 주저앉을 수 있으니 잡은 뒤 진행하세요.", False)):
        safe_torque_off(bus, motors)
        print("토크를 해제했습니다.")
    else:
        print("토크를 유지하고 있습니다. 해제하려면: python -m tools.pose release")
    result = {"pose": args.name, "duration": args.duration, "hold": args.hold,
              "torque_pct": args.torque_pct, "stats": stats}
    print(f"로그: {save_log('pose_' + args.name, result)}")
    return result


def cmd_monitor(bus, cfg, args) -> None:
    motors = present_motors(bus, range(0, 31), include_wheel=True)
    n = 0
    try:
        while args.count is None or n < args.count:
            rows = sample_all(bus, motors)
            print("\033[2J\033[H" if args.count is None else "", end="")
            print(f"{'ID':>3} {'NAME':<17} {'POS(deg)':>8} {'LOAD%':>6} {'TEMP':>5} {'VOLT':>5} {'TQ':>3}  ERR")
            for i, live in rows.items():
                if live is None:
                    print(f"{i:>3} {cfg.name_of(i):<17} 통신 실패")
                    continue
                print(f"{i:>3} {cfg.name_of(i):<17} {live['position_deg']:>8.1f} {live['load_pct']:>6.0f} "
                      f"{live['temperature']:>5} {live['voltage']:>5.1f} {live['torque_enable']:>3}  "
                      f"{','.join(live['errors'])}")
            n += 1
            bus.sleep(args.interval)
    except KeyboardInterrupt:
        pass


def cmd_release(bus, cfg, args) -> None:
    motors = present_motors(bus, range(0, 31), include_wheel=True)
    if confirm(f"{len(motors)}개 모터의 토크를 해제합니다. 로봇을 잡으세요.", args.yes):
        safe_torque_off(bus, motors)
        print("토크를 해제했습니다.")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("name")
    r.add_argument("--keep-torque", action="store_true",
                   help="record the current pose without turning torque off (e.g. pose left by CM-530)")
    pl = sub.add_parser("play")
    pl.add_argument("name")
    pl.add_argument("--duration", type=float, default=2.0, help="seconds to reach the pose")
    pl.add_argument("--hold", type=float, default=5.0, help="seconds to hold while logging")
    pl.add_argument("--torque-pct", type=float, default=70.0)
    pl.add_argument("--release", action="store_true", help="turn torque off after holding")
    mo = sub.add_parser("monitor")
    mo.add_argument("--interval", type=float, default=0.5)
    mo.add_argument("--count", type=int, help="stop after N refreshes")
    sub.add_parser("release")
    sub.add_parser("list")
    args = p.parse_args(argv)

    if args.cmd == "list":
        for f in sorted(POSE_DIR.glob("*.yaml")):
            print(f.stem)
        return None

    cfg = load_config(args.config)
    bus = open_bus(args, cfg)
    try:
        return {"record": cmd_record, "play": cmd_play, "monitor": cmd_monitor,
                "release": cmd_release}[args.cmd](bus, cfg, args)
    except KeyboardInterrupt:
        return None
    finally:
        bus.close()


if __name__ == "__main__":
    main()
