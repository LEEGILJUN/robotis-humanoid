"""Test every servo one by one: health registers, holding noise and a small +/- motion.

Lay the robot on its back (or hang it) so each joint can move about 10 degrees freely.

    python -m tools.motor_test                 # every motor found on IDs 0-30, asks before each
    python -m tools.motor_test --ids 13 14 -y  # only the knees, no prompts
    python -m tools.motor_test --skip-motion   # registers only, nothing moves
    python -m tools.motor_test --port sim --sim-faults -y
"""
from __future__ import annotations

import argparse

from humanoid.bus import CommError
from humanoid.control_table import ADDR, MODELS
from humanoid.robot import (
    add_common_args, confirm, load_config, open_bus, safe_torque_off, save_log,
)
from humanoid.status import FAIL, WARN, health_issues, read_live, read_status, verdict


def motion_test(bus, dxl_id: int, st, model, t: dict) -> dict:
    scale = model.resolution // 1024
    amp = round(t["amplitude_deg"] / model.deg_per_tick)
    lo, hi = st.cw_angle_limit + 5 * scale, st.ccw_angle_limit - 5 * scale
    clamp = lambda v: max(lo, min(hi, v))  # noqa: E731
    start = st.position
    targets = [clamp(start + amp), clamp(start - amp), clamp(start)]
    tol = max(2 * scale, round(t["max_error_deg"] / model.deg_per_tick / 2))
    torque_limit = max(1, round(1023 * t["torque_limit_pct"] / 100))
    speed_raw = max(1, round(t["speed_rpm"] / model.speed_unit_rpm))
    restore_limit = st.torque_limit if st.torque_limit > 0 else st.max_torque

    out = {"jitter_ticks": None, "steps": []}
    try:
        bus.write(dxl_id, *ADDR["goal_position"], start)
        bus.write(dxl_id, *ADDR["torque_limit"], torque_limit)
        bus.write(dxl_id, *ADDR["moving_speed"], speed_raw)
        bus.write(dxl_id, *ADDR["torque_enable"], 1)
        bus.sleep(0.3)

        samples = []
        for _ in range(20):
            samples.append(read_live(bus, dxl_id, model)["position"])
            bus.sleep(0.01)
        out["jitter_ticks"] = (max(samples) - min(samples)) // scale

        for target in targets:
            bus.write(dxl_id, *ADDR["goal_position"], target)
            t0 = bus.now()
            peak, errors = 0.0, set()
            while True:
                live = read_live(bus, dxl_id, model)
                elapsed = bus.now() - t0
                peak = max(peak, abs(live["load_pct"]))
                errors |= set(live["errors"])
                if errors & {"overload", "overheating"}:
                    break
                if not live["moving"] and abs(live["position"] - target) <= tol:
                    break
                if not live["moving"] and elapsed > 0.5:  # settled short of the target
                    break
                if elapsed > t["step_timeout_s"]:
                    break
                bus.sleep(0.02)
            out["steps"].append({
                "target_deg": round(model.ticks_to_deg(target), 1),
                "final_deg": live["position_deg"],
                "error_deg": round(abs(live["position"] - target) * model.deg_per_tick, 2),
                "time_s": round(elapsed, 2),
                "peak_load_pct": peak,
                "errors": sorted(errors),
            })
            if errors & {"overload", "overheating"}:
                break
    finally:
        safe_torque_off(bus, [dxl_id])
        try:
            bus.write(dxl_id, *ADDR["torque_limit"], restore_limit)
        except CommError:
            pass
    return out


def judge_motion(m: dict, t: dict) -> list[tuple[str, str]]:
    issues = []
    for s in m["steps"]:
        if s["errors"]:
            issues.append((FAIL, f"동작 중 에러 {s['errors']} → 기구 간섭이나 기어 파손 의심, 전원을 껐다 켜서 복구"))
            break
    worst = max((s["error_deg"] for s in m["steps"]), default=0.0)
    if worst > 2 * t["max_error_deg"]:
        issues.append((FAIL, f"목표 위치 미도달 (최대 오차 {worst:.1f}°) → 기어 손상, 간섭, 토크 부족 의심"))
    elif worst > t["max_error_deg"]:
        issues.append((WARN, f"추종 오차 큼 (최대 {worst:.1f}°)"))
    if m["jitter_ticks"] is not None and m["jitter_ticks"] > t["max_jitter_ticks"]:
        issues.append((WARN, f"정지 중 위치값 흔들림 {m['jitter_ticks']} tick → 위치 센서(가변저항) 마모 의심"))
    peak = max((s["peak_load_pct"] for s in m["steps"]), default=0.0)
    if peak > 80:
        issues.append((WARN, f"부하 높음 (최대 {peak:.0f}%)"))
    return issues


def main(argv=None) -> list[dict]:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--ids", type=int, nargs="+", help="IDs to test (default: every motor found on 0-30)")
    p.add_argument("--skip-motion", action="store_true", help="read registers only")
    p.add_argument("--amplitude", type=float, help="override amplitude_deg")
    p.add_argument("--torque-pct", type=float, help="override torque_limit_pct")
    p.add_argument("--force", action="store_true", help="run the motion test even after a FAIL")
    args = p.parse_args(argv)

    cfg = load_config(args.config)
    t = dict(cfg.test)
    t.setdefault("amplitude_deg", 10)
    t.setdefault("torque_limit_pct", 40)
    t.setdefault("speed_rpm", 12)
    t.setdefault("max_error_deg", 3.0)
    t.setdefault("max_jitter_ticks", 3)
    t.setdefault("step_timeout_s", 3.0)
    if args.amplitude is not None:
        t["amplitude_deg"] = args.amplitude
    if args.torque_pct is not None:
        t["torque_limit_pct"] = args.torque_pct

    bus = open_bus(args, cfg)
    results = []
    try:
        candidates = args.ids or range(0, 31)
        found = [i for i in candidates if bus.ping(i) is not None]
        missing = [i for i in (args.ids or cfg.joints) if i not in found]
        print(f"시험 대상 {len(found)}개: {found}")
        if missing:
            print(f"응답 없음: {', '.join(f'{i}({cfg.name_of(i)})' for i in missing)}")
        if not args.skip_motion:
            print(f"각 모터를 현재 위치 기준 ±{t['amplitude_deg']}°, 토크 {t['torque_limit_pct']}%로 움직입니다. "
                  "로봇을 눕히거나 매달아 두세요.\n")

        for dxl_id in found:
            name = cfg.name_of(dxl_id)
            joint = cfg.joints.get(dxl_id)
            row = {"id": dxl_id, "name": name}
            try:
                st = read_status(bus, dxl_id)
                issues = health_issues(st, joint.model if joint else None, t)
                row["status"] = st.to_dict()
                model = MODELS.get(st.model_number)
                run_motion = not args.skip_motion and model is not None
                if run_motion and st.cw_angle_limit == 0 and st.ccw_angle_limit == 0:
                    issues.append((WARN, "Wheel 모드라 동작 시험을 건너뜀 (토크를 켜면 계속 회전할 수 있음)"))
                    run_motion = False
                if run_motion and verdict(issues) == FAIL and not args.force:
                    issues.append((WARN, "사전 점검 FAIL이라 동작 시험을 건너뜀 (--force 로 강제 실행)"))
                    run_motion = False
                if run_motion and confirm(f"ID {dxl_id} ({name}, {st.model_name}) 움직입니다.", args.yes):
                    m = motion_test(bus, dxl_id, st, model, t)
                    row["motion"] = m
                    issues += judge_motion(m, t)
            except CommError as e:
                issues = [(FAIL, f"통신 실패: {e}")]
            row["issues"] = issues
            row["verdict"] = verdict(issues)
            results.append(row)

            m = row.get("motion")
            extra = ""
            if m:
                worst = max((s["error_deg"] for s in m["steps"]), default=0)
                peak = max((s["peak_load_pct"] for s in m["steps"]), default=0)
                extra = f"err {worst:4.1f}°  load {peak:3.0f}%  jitter {m['jitter_ticks']}"
            print(f"{dxl_id:>3} {name:<17} {row['verdict']:<4}  {extra}")
            for lvl, msg in issues:
                print(f"      - [{lvl}] {msg}")

        for i in missing:
            results.append({"id": i, "name": cfg.name_of(i), "verdict": FAIL, "issues": [(FAIL, "응답 없음")]})

        counts = {k: sum(r["verdict"] == k for r in results) for k in ("PASS", "WARN", "FAIL")}
        print(f"\n판정: PASS {counts['PASS']} / WARN {counts['WARN']} / FAIL {counts['FAIL']}")
        print(f"로그: {save_log('motor_test', results)}")
        return results
    except KeyboardInterrupt:
        print("\n중단했습니다. 모든 모터의 토크를 해제합니다.")
        safe_torque_off(bus, range(0, 31))
        return results
    finally:
        bus.close()


if __name__ == "__main__":
    main()
