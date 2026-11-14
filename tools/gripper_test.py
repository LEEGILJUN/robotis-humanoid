"""Open/close test for the wheel-mode AX-18A grippers.

Start with every gripper fully CLOSED. The position sensor of these grippers passes through
the AX dead zone (300-360 deg), so the end stops are found from the measured speed instead:
the motor is driven with a small output ratio (in wheel mode Moving Speed is an output
ratio, not an rpm) and stops as soon as it has not moved for a short time.

    python -m tools.gripper_test            # IDs 19 and 20
    python -m tools.gripper_test --ids 20
    python -m tools.gripper_test --port sim -y
"""
from __future__ import annotations

import argparse

from humanoid.bus import CommError
from humanoid.control_table import ADDR, MODELS
from humanoid.robot import add_common_args, confirm, load_config, open_bus, save_log
from humanoid.status import is_wheel_mode, read_live, read_status

CCW, CW = 1, -1


def run_wheel(bus, gid: int, model, direction: int, p: dict, max_time: float, max_travel: float) -> dict:
    """Turn in one direction until stall, travel limit or timeout. Always stops and releases."""
    output_raw = max(1, round(1023 * p["output_pct"] / 100))
    torque_limit = max(output_raw, round(1023 * p["torque_pct"] / 100))
    command = output_raw | (0x400 if direction == CW else 0)
    st = read_status(bus, gid)
    restore_limit = st.torque_limit if st.torque_limit > 0 else st.max_torque

    travel, peak, trace = 0.0, 0.0, []
    result, errors = "timeout", set()
    start_pos = st.position
    try:
        bus.write(gid, *ADDR["moving_speed"], 0)
        bus.write(gid, *ADDR["torque_limit"], torque_limit)
        bus.write(gid, *ADDR["torque_enable"], 1)
        bus.write(gid, *ADDR["moving_speed"], command)
        t0 = last = bus.now()
        stall_since = None
        while True:
            live = read_live(bus, gid, model)
            now = bus.now()
            dt, last = now - last, now
            rpm = abs(live["speed_raw"]) * model.speed_unit_rpm
            travel += rpm * 6.0 * dt
            peak = max(peak, abs(live["load_pct"]))
            errors |= set(live["errors"])
            trace.append((round(now - t0, 3), live["position"], live["load_pct"], live["speed_raw"]))
            if errors & {"overload", "overheating"}:
                result = "overload"
                break
            stopped = abs(live["speed_raw"]) <= p["stopped_speed_raw"]
            if now - t0 > 0.3 and stopped:
                stall_since = stall_since if stall_since is not None else now
                if now - stall_since >= p["stall_confirm_s"]:
                    result = "stall"
                    break
            else:
                stall_since = None
            if travel >= max_travel:
                result = "travel_limit"
                break
            if now - t0 > max_time:
                break
            bus.sleep(0.01)
    finally:
        for name, value in (("moving_speed", 0), ("torque_enable", 0), ("torque_limit", restore_limit)):
            try:
                bus.write(gid, *ADDR[name], value)
            except CommError:
                pass
    jumps = sum(1 for a, b in zip(trace, trace[1:]) if abs(b[1] - a[1]) > 100)
    return {
        "direction": "CCW" if direction == CCW else "CW",
        "result": result,
        "travel_deg": round(travel, 1),
        "peak_load_pct": peak,
        "start_pos": start_pos,
        "end_pos": trace[-1][1] if trace else start_pos,
        "position_jumps": jumps,  # > 0 means the travel crossed the potentiometer dead zone
        "errors": sorted(errors),
        "samples": len(trace),
    }


def test_gripper(bus, gid: int, p: dict) -> dict:
    model = MODELS[bus.ping(gid)]
    out = {"id": gid, "model": model.name}
    # 1. Probe: a short turn CW. Fully closed + immediate stall -> CW is the closing direction.
    probe = run_wheel(bus, gid, model, CW, p, max_time=p["probe_s"], max_travel=p["probe_deg"])
    out["probe"] = probe
    if probe["result"] in ("stall", "overload") and probe["travel_deg"] < p["probe_deg"] * 0.5:
        open_dir, already = CCW, 0.0
    else:
        open_dir, already = CW, probe["travel_deg"]
    close_dir = -open_dir
    out["open_direction"] = "CCW" if open_dir == CCW else "CW"
    bus.sleep(0.3)
    # 2. Open until the open stop.
    opening = run_wheel(bus, gid, model, open_dir, p, max_time=p["max_time_s"], max_travel=p["max_travel_deg"])
    out["open"] = opening
    bus.sleep(0.3)
    # 3. Close until the closed stop.
    closing = run_wheel(bus, gid, model, close_dir, p, max_time=p["max_time_s"], max_travel=p["max_travel_deg"])
    out["close"] = closing
    out["open_travel_deg"] = round(already + opening["travel_deg"], 1)
    out["close_travel_deg"] = closing["travel_deg"]

    issues = []
    if opening["result"] != "stall":
        issues.append(f"열림 끝을 찾지 못함 ({opening['result']})")
    if closing["result"] != "stall":
        issues.append(f"닫힘 끝을 찾지 못함 ({closing['result']})")
    if any(r["errors"] for r in (probe, opening, closing)):
        issues.append("동작 중 에러 비트 발생")
    if opening["result"] == "stall" and closing["result"] == "stall":
        diff = abs(out["open_travel_deg"] - out["close_travel_deg"])
        if diff > 20:
            issues.append(f"열림/닫힘 이동량 차이 {diff:.0f}° (미끄러짐이나 걸림 의심)")
    out["dead_zone_crossed"] = any(r["position_jumps"] for r in (opening, closing))
    out["fits_joint_mode"] = out["open_travel_deg"] < model.range_deg - 20
    out["issues"] = issues
    out["verdict"] = "FAIL" if issues else "PASS"
    return out


def main(argv=None) -> list[dict]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("--ids", type=int, nargs="+", default=[19, 20])
    ap.add_argument("--output-pct", type=float, default=10.0, help="wheel-mode output ratio")
    ap.add_argument("--torque-pct", type=float, default=30.0, help="torque limit (caps the output)")
    args = ap.parse_args(argv)
    p = {"output_pct": args.output_pct, "torque_pct": args.torque_pct, "stopped_speed_raw": 3,
         "stall_confirm_s": 0.3, "probe_s": 0.8, "probe_deg": 15.0, "max_time_s": 10.0, "max_travel_deg": 330.0}

    cfg = load_config(args.config)
    bus = open_bus(args, cfg)
    results = []
    try:
        for gid in args.ids:
            if bus.ping(gid) is None:
                print(f"ID {gid}: 응답 없음")
                continue
            if not is_wheel_mode(bus, gid):
                print(f"ID {gid}: Wheel 모드가 아니라서 건너뜀 (이 도구는 Wheel 모드 그리퍼 전용)")
                continue
            if not confirm(f"ID {gid} ({cfg.name_of(gid)}) 를 열었다 닫습니다. 완전히 닫힌 상태인지 확인하세요.", args.yes):
                continue
            r = test_gripper(bus, gid, p)
            results.append(r)
            print(f"ID {gid} {cfg.name_of(gid):<10} {r['verdict']}  열림 방향 {r['open_direction']}, "
                  f"열림 {r['open_travel_deg']}° / 닫힘 {r['close_travel_deg']}°, "
                  f"최대 부하 {max(r['open']['peak_load_pct'], r['close']['peak_load_pct']):.0f}%")
            print(f"      사각 구간 통과: {'예' if r['dead_zone_crossed'] else '아니오'}, "
                  f"관절 모드로 바꿀 수 있는 이동량: {'예' if r['fits_joint_mode'] else '아니오'}")
            for msg in r["issues"]:
                print(f"      - {msg}")
        print(f"로그: {save_log('gripper_test', results)}")
        return results
    finally:
        bus.close()


if __name__ == "__main__":
    main()
