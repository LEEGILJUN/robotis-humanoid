"""Scan the DYNAMIXEL bus, read every servo's state and compare it with the config.

    python -m tools.scan                    # IDs 0-30 at the configured baudrate
    python -m tools.scan --all-bauds --full # every common baudrate, IDs 0-252 (slow)
    python -m tools.scan --blink 19 20      # blink LEDs to find which motor is which
    python -m tools.scan --port sim --sim-faults
"""
from __future__ import annotations

import argparse

from humanoid.bus import CommError
from humanoid.control_table import COMMON_BAUDRATES, MODELS
from humanoid.robot import add_common_args, load_config, open_bus, save_log
from humanoid.status import health_issues, read_status, verdict


def blink(bus, ids: list[int], times: int = 8) -> None:
    for dxl_id in ids:
        print(f"ID {dxl_id}: LED 깜빡이는 중... 어느 모터인지 확인하세요.")
        try:
            for _ in range(times):
                bus.write(dxl_id, 25, 1, 1)
                bus.sleep(0.25)
                bus.write(dxl_id, 25, 1, 0)
                bus.sleep(0.25)
        except CommError as e:
            print(f"  {e}")


def scan_ids(bus, ids) -> dict[int, int]:
    found = {}
    for dxl_id in ids:
        model = bus.ping(dxl_id)
        if model is not None:
            found[dxl_id] = model
    return found


def main(argv=None) -> dict:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("--full", action="store_true", help="scan IDs 0-252 instead of 0-30")
    p.add_argument("--all-bauds", action="store_true", help="try every common baudrate")
    p.add_argument("--blink", type=int, nargs="+", metavar="ID", help="blink LEDs of these IDs and exit")
    args = p.parse_args(argv)

    cfg = load_config(args.config)
    bus = open_bus(args, cfg)
    try:
        if args.blink:
            blink(bus, args.blink)
            return {}

        ids = range(0, 253 if args.full else 31)
        bauds = COMMON_BAUDRATES if args.all_bauds else [args.baud or cfg.baudrate]
        found_by_baud: dict[int, dict[int, int]] = {}
        for baud in bauds:
            try:
                bus.set_baudrate(baud)
            except CommError:
                print(f"건너뜀: {baud} bps (이 포트에서 설정할 수 없는 속도)")
                continue
            print(f"스캔 중: {baud} bps, ID {ids.start}-{ids.stop - 1} ...")
            found = scan_ids(bus, ids)
            if found:
                found_by_baud[baud] = found
                print(f"  {len(found)}개 발견")

        main_baud = cfg.baudrate if cfg.baudrate in found_by_baud else next(iter(found_by_baud), None)
        rows = []
        if main_baud is not None:
            bus.set_baudrate(main_baud)
            print(f"\n[{main_baud} bps 상세]")
            print(f"{'ID':>3} {'NAME':<17} {'MODEL':<8} {'FW':>3} {'VOLT':>5} {'TEMP':>4} "
                  f"{'POS(deg)':>8} {'TQLIM':>5}  RESULT")
            for dxl_id in sorted(found_by_baud[main_baud]):
                joint = cfg.joints.get(dxl_id)
                try:
                    st = read_status(bus, dxl_id)
                except CommError as e:
                    print(f"{dxl_id:>3} 읽기 실패: {e}")
                    rows.append({"id": dxl_id, "verdict": "FAIL", "issues": [["FAIL", str(e)]]})
                    continue
                issues = health_issues(st, joint.model if joint else None, cfg.test)
                v = verdict(issues)
                print(f"{dxl_id:>3} {cfg.name_of(dxl_id):<17} {st.model_name:<8} {st.firmware:>3} "
                      f"{st.voltage:>5.1f} {st.temperature:>4} {st.position_deg:>8.1f} {st.torque_limit:>5}  {v}")
                for lvl, msg in issues:
                    print(f"      - [{lvl}] {msg}")
                rows.append({"id": dxl_id, "name": cfg.name_of(dxl_id), "verdict": v,
                             "issues": issues, "status": st.to_dict()})

        found_main = set(found_by_baud.get(main_baud, {})) if main_baud else set()
        expected = set(cfg.joints)
        missing = sorted(expected - found_main)
        unexpected = sorted(found_main - expected)
        other_baud = {b: f for b, f in found_by_baud.items() if b != main_baud}

        print("\n[요약]")
        print(f"  설정된 관절 {len(expected)}개 중 응답 {len(expected & found_main)}개")
        if missing:
            print(f"  응답 없음: {', '.join(f'{i}({cfg.name_of(i)})' for i in missing)}"
                  "  → 케이블, 커넥터, ID 중복을 확인하세요")
        if unexpected:
            desc = ", ".join(f"{i}({MODELS[m].name if m in MODELS else m})"
                             for i, m in sorted(found_by_baud[main_baud].items()) if i in unexpected)
            print(f"  설정에 없는 ID: {desc}  → 그리퍼라면 --blink 로 위치를 확인한 뒤 config에 추가하세요")
        for b, f in other_baud.items():
            print(f"  다른 보드레이트({b} bps)에서 발견: {sorted(f)}  → 같은 버스에서 쓰려면 보드레이트를 맞춰야 합니다")
        counts = {k: sum(r["verdict"] == k for r in rows) for k in ("PASS", "WARN", "FAIL")}
        print(f"  판정: PASS {counts['PASS']} / WARN {counts['WARN']} / FAIL {counts['FAIL']}")

        result = {"port": bus.port_name, "found": {str(b): f for b, f in found_by_baud.items()},
                  "missing": missing, "unexpected": unexpected, "rows": rows}
        print(f"  로그: {save_log('scan', result)}")
        return result
    finally:
        bus.close()


if __name__ == "__main__":
    main()
