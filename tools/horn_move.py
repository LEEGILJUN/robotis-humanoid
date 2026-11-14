"""Move a (detached) horn to an absolute position in joint mode, with stall protection.

Used when re-mounting a gripper: detach the gripper, move the horn, re-attach.
1. Probe: 10 deg toward the target at 10 % torque. Any stall here means something is still
   attached, and the tool stops.
2. Move: the rest of the way at --torque-pct, stopping within --stall-s of a stall.
Torque is always turned off at the end.

    python -m tools.horn_move 19 956
"""
from __future__ import annotations

import argparse

from humanoid.bus import CommError
from humanoid.control_table import ADDR, MODELS
from humanoid.robot import add_common_args, confirm, load_config, open_bus
from humanoid.status import is_wheel_mode, read_live, read_status


def move(bus, gid: int, model, goal: int, torque_pct: float, speed_raw: int, stall_s: float, timeout: float) -> dict:
    limit = max(1, round(1023 * torque_pct / 100))
    bus.write(gid, *ADDR["torque_limit"], limit)
    bus.write(gid, *ADDR["moving_speed"], speed_raw)
    bus.write(gid, *ADDR["goal_position"], goal)
    t0 = bus.now()
    still_since, last_pos, peak = None, None, 0.0
    while True:
        d = read_live(bus, gid, model)
        now = bus.now()
        peak = max(peak, abs(d["load_pct"]))
        at_goal = abs(d["position"] - goal) <= 2
        if at_goal and not d["moving"]:
            return {"result": "reached", "pos": d["position"], "peak_load": peak, "time": round(now - t0, 2)}
        moved = last_pos is None or abs(d["position"] - last_pos) > 1
        last_pos = d["position"]
        if not moved and now - t0 > 0.3:
            still_since = still_since if still_since is not None else now
            if now - still_since >= stall_s:
                # The Moving flag can lag; standing still at the goal counts as reached.
                result = "reached" if at_goal else "stall"
                return {"result": result, "pos": d["position"], "peak_load": peak, "time": round(now - t0, 2)}
        else:
            still_since = None
        if d["errors"]:
            return {"result": f"error {d['errors']}", "pos": d["position"], "peak_load": peak, "time": round(now - t0, 2)}
        if now - t0 > timeout:
            return {"result": "timeout", "pos": d["position"], "peak_load": peak, "time": round(now - t0, 2)}
        bus.sleep(0.03)


def main(argv=None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(ap)
    ap.add_argument("id", type=int)
    ap.add_argument("goal", type=int)
    ap.add_argument("--torque-pct", type=float, default=30.0)
    ap.add_argument("--speed", type=int, default=40, help="Moving Speed (joint mode, ~0.111 rpm/unit)")
    ap.add_argument("--stall-s", type=float, default=0.3)
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    bus = open_bus(args, cfg)
    gid = args.id
    try:
        number = bus.ping(gid)
        if number not in MODELS:
            print(f"ID {gid} 응답 없음")
            return {"result": "no_response"}
        model = MODELS[number]
        if is_wheel_mode(bus, gid):
            print(f"ID {gid}는 Wheel 모드입니다. 먼저 관절 모드로 바꾸세요.")
            return {"result": "wheel_mode"}
        st = read_status(bus, gid)
        lo, hi = st.cw_angle_limit, st.ccw_angle_limit
        if not lo <= args.goal <= hi:
            print(f"목표 {args.goal}가 각도 제한 {lo}~{hi} 밖입니다.")
            return {"result": "out_of_limits"}
        if not confirm(f"ID {gid} 혼을 {st.position} → {args.goal} 로 옮깁니다. 그리퍼가 완전히 분리되었나요?", args.yes):
            return {"result": "cancelled"}
        direction = 1 if args.goal > st.position else -1
        probe_goal = st.position + direction * min(34, abs(args.goal - st.position))  # ~10 deg
        bus.write(gid, *ADDR["goal_position"], st.position)
        bus.write(gid, *ADDR["torque_enable"], 1)
        print(f"시작 위치 {st.position}, 탐색 목표 {probe_goal}, 최종 목표 {args.goal}")
        probe = move(bus, gid, model, probe_goal, 10.0, args.speed, args.stall_s, timeout=5)
        print(f"탐색 이동: {probe}")
        if probe["result"] != "reached":
            print("탐색 단계에서 멈췄습니다. 혼에 무언가 연결되어 있거나 걸려 있는지 확인하세요.")
            return {"result": "probe_failed", "probe": probe}
        main_move = move(bus, gid, model, args.goal, args.torque_pct, args.speed, args.stall_s, timeout=30)
        print(f"본 이동: {main_move}")
        return {"result": main_move["result"], "probe": probe, "move": main_move}
    finally:
        for name, value in (("torque_enable", 0), ("torque_limit", 1023)):
            try:
                bus.write(gid, *ADDR[name], value)
            except CommError:
                pass
        bus.close()


if __name__ == "__main__":
    main()
