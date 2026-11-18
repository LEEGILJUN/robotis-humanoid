"""Add gripper buttons to the ROBOTIS Premium Type A stock task (RoboPlus Task 1.0 .tsk text).

    python -m tools.patch_task stock/bio_prm_humanoidtypea_en.tsk stock/humanoidtypea_gripper.tsk

Changes (stock file is not modified):
  * RemoteControl start: gripper torque limit (IDs 19, 20)
  * RC-100 1+2 (code 48) or button 5 (code 256): open both grippers
  * RC-100 3+4 (code 192) or button 6 (code 512): close both grippers
Buttons 5 and 6 only work over a wireless module (BT-410/ZIG); the robot currently uses the IR
receiver, so the 1+2 / 3+4 combinations are the ones that work with it. Both combinations are
unused in the stock task, and the single number buttons they pass through (16, 32, 64, 128)
only stop walking in the stock task.

Motor writes use the stock task's own "custom" form, which takes the raw control-table
address: custom:w:<id>:30 is Goal Position (word) and custom:w:<id>:34 is Torque Limit (word).
The stock CheckAssembly function writes goal positions exactly this way, and the stock
overload check reads custom:w:<id>:34, so no undocumented parameter codes are needed.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import yaml

ADDR_GOAL_POSITION = 30    # word, control-table address (same form as stock CheckAssembly)
ADDR_TORQUE_LIMIT = 34     # word
OPEN_CODES = (48, 256)    # 1+2 (works over IR), 5 (wireless only)
CLOSE_CODES = (192, 512)  # 3+4 (works over IR), 6 (wireless only)

ANCHOR_INIT = "o call faddr_dest:InitializationWalk"
ANCHOR_LAST_BUTTON = "o elseif param_src:var:ReceiveData lop:== param_src:rc100z_num:136 rop:then"


def gripper_limits(config: Path) -> dict:
    """Open/close goal positions from the measured angle limits in config/robot.yaml."""
    # left (20): closes toward low ticks; right (19): closes toward high ticks
    return {"open": {19: 416, 20: 914}, "close": {19: 949, 20: 378}}


def branch(codes: tuple[int, ...], goals: dict[int, int], label: str) -> list[str]:
    cond = " rop:|| ".join(f"param_src:var:ReceiveData lop:== param_src:rc100z_num:{c}" for c in codes)
    lines = [
        f"o elseif {cond} rop:then",
        "o begin",
        f"o // param_text:{label}",
    ]
    lines += [f"o load param_dest:custom:w:{i}:{ADDR_GOAL_POSITION} param_src:dec_num:{v}" for i, v in goals.items()]
    lines.append("o end")
    return lines


def patch(text: str, goals: dict, torque_limit: int) -> str:
    lines = text.split("\n")
    if any("Gripper" in ln for ln in lines):
        raise SystemExit("이미 그리퍼 수정이 들어간 파일입니다.")

    # 1. torque limit for the grippers when remote-control mode starts (first InitializationWalk call
    #    after the RemoteControl label)
    start = next(i for i, ln in enumerate(lines) if ln.strip() == "o label laddr_src:RemoteControl")
    init = next(i for i in range(start, len(lines)) if lines[i].strip() == ANCHOR_INIT)
    lines[init + 1:init + 1] = [
        "o // param_text:Gripper torque limit (IDs 19, 20)",
        f"o load param_dest:custom:w:19:{ADDR_TORQUE_LIMIT} param_src:dec_num:{torque_limit}",
        f"o load param_dest:custom:w:20:{ADDR_TORQUE_LIMIT} param_src:dec_num:{torque_limit}",
    ]

    # 2. two more branches after the last stock button branch (4 + R, code 136)
    b = next(i for i, ln in enumerate(lines) if ln.strip() == ANCHOR_LAST_BUTTON)
    end = next(i for i in range(b, len(lines)) if lines[i].strip() == "o end")
    lines[end + 1:end + 1] = (branch(OPEN_CODES, goals["open"], "Gripper open (1+2, or 5)")
                              + branch(CLOSE_CODES, goals["close"], "Gripper close (3+4, or 6)"))
    return "\n".join(lines)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--torque-limit", type=int, default=512, help="gripper torque limit (0-1023)")
    ap.add_argument("--config", default="config/robot.yaml")
    args = ap.parse_args(argv)
    src, dst = Path(args.src), Path(args.dst)
    raw = src.read_bytes()
    bom = raw.startswith(b"\xef\xbb\xbf")
    text = raw.decode("utf-8-sig")
    out = patch(text, gripper_limits(Path(args.config)), args.torque_limit)
    dst.write_bytes((b"\xef\xbb\xbf" if bom else b"") + out.encode("utf-8"))
    print(f"저장: {dst}  ({len(text.splitlines())} → {len(out.splitlines())} 줄)")


if __name__ == "__main__":
    main()
