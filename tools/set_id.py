"""Change a servo's ID (EEPROM). Connect only the chain that holds the motor if its ID may be duplicated.

    python -m tools.set_id 1 20 --expect-model AX-18A
"""
from __future__ import annotations

import argparse
import sys

from humanoid.control_table import ADDR, MODELS, MODELS_BY_NAME
from humanoid.robot import add_common_args, confirm, load_config, open_bus


def main(argv=None) -> bool:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p)
    p.add_argument("old_id", type=int)
    p.add_argument("new_id", type=int)
    p.add_argument("--expect-model", help="refuse unless the motor at old_id is this model (e.g. AX-18A)")
    args = p.parse_args(argv)
    if not (0 <= args.new_id <= 252):
        sys.exit("새 ID는 0~252 범위여야 합니다.")

    cfg = load_config(args.config)
    bus = open_bus(args, cfg)
    try:
        number = bus.ping(args.old_id)
        if number is None:
            print(f"ID {args.old_id} 응답 없음")
            return False
        name = MODELS[number].name if number in MODELS else str(number)
        if args.expect_model and MODELS_BY_NAME.get(args.expect_model, None) and number != MODELS_BY_NAME[args.expect_model].number:
            print(f"ID {args.old_id}의 모델이 {name}이라 중단합니다 (예상: {args.expect_model}).")
            return False
        if bus.ping(args.new_id) is not None:
            print(f"ID {args.new_id}는 이미 사용 중이라 중단합니다.")
            return False
        if not confirm(f"ID {args.old_id} ({name}) → ID {args.new_id} 로 변경합니다.", args.yes):
            return False
        bus.write(args.old_id, 24, 1, 0)  # torque off before touching EEPROM
        try:
            bus.write(args.old_id, *ADDR["id"], args.new_id)
        except Exception:
            pass  # the status packet may come back from the new ID; verify below instead
        bus.sleep(0.1)
        ok = bus.ping(args.new_id) == number and bus.ping(args.old_id) is None
        print("변경 완료" if ok else "변경 확인 실패: 스캔으로 상태를 확인하세요")
        return ok
    finally:
        bus.close()


if __name__ == "__main__":
    main()
