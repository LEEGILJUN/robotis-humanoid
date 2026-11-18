"""RC-100 button codes and packets. Sending goes through tools.bt_remote.

WARNING: do not send through /dev/cu.ROBOTISBT-210 on macOS. Opening it does not bring the
link up, and the bytes are queued and delivered all at once the next time the BT-210 connects
(2026-11-15: queued walk commands made the robot stand up and walk unexpectedly).

Original description: act as an RC-100 remote over the robot's BT-210 (57600 bps).

The CM-530 task reads remote data the same way whether it comes from the RC-100 or from here.
Packet: 0xFF 0x55 DataL ~DataL DataH ~DataH  (button codes are summed).

    python -m tools.rc100 tap 2+U              # one press (e.g. 2+U = pound chest)
    python -m tools.rc100 hold U 2.0           # walk forward for 2 s, then release
    python -m tools.rc100 tap 1+2              # gripper open  (patched task)
    python -m tools.rc100 tap 3+4              # gripper close (patched task)
"""
from __future__ import annotations

import argparse
import glob
import sys
import time

BUTTONS = {"U": 1, "D": 2, "L": 4, "R": 8, "1": 16, "2": 32, "3": 64, "4": 128, "5": 256, "6": 512}


def code_of(combo: str) -> int:
    """'2+U' -> 33. Plain numbers ('33') are taken as raw codes."""
    if combo.isdigit() and combo not in BUTTONS:
        return int(combo)
    total = 0
    for part in combo.upper().split("+"):
        if part not in BUTTONS:
            raise ValueError(f"알 수 없는 버튼: {part} (사용 가능: {', '.join(BUTTONS)})")
        total += BUTTONS[part]
    return total


def packet(code: int) -> bytes:
    lo, hi = code & 0xFF, (code >> 8) & 0xFF
    return bytes([0xFF, 0x55, lo, ~lo & 0xFF, hi, ~hi & 0xFF])


def default_port() -> str | None:
    ports = glob.glob("/dev/cu.ROBOTISBT-*") + glob.glob("/dev/cu.*BT-210*")
    return ports[0] if ports else None


class Remote:
    def __init__(self, port: str, baud: int = 57600, connect_wait: float = 3.0):
        import serial

        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=5)
        # Opening a Bluetooth serial port only starts the radio link; bytes written before
        # the link is up are lost. Wait for it (the BT-210 LED turns solid when connected).
        time.sleep(connect_wait)

    def send(self, code: int, times: int = 1, interval: float = 0.05) -> None:
        for _ in range(times):
            self.ser.write(packet(code))
            self.ser.flush()
            time.sleep(interval)

    def hold(self, code: int, seconds: float, interval: float = 0.05) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(code, 1, interval)
        self.release()

    def release(self) -> None:
        self.send(0, times=3)

    def close(self) -> None:
        time.sleep(0.5)  # let the last packets leave before the radio link is dropped
        self.ser.close()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="Bluetooth serial port (default: /dev/cu.ROBOTISBT-*)")
    ap.add_argument("--connect-wait", type=float, default=3.0, help="seconds to wait for the Bluetooth link")
    sub = ap.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("tap")
    t.add_argument("combo")
    h = sub.add_parser("hold")
    h.add_argument("combo")
    h.add_argument("seconds", type=float)
    sub.add_parser("show").add_argument("combo")
    args = ap.parse_args(argv)

    code = code_of(args.combo)
    if args.cmd == "show":
        print(f"{args.combo} = {code}, packet {packet(code).hex(' ')}")
        return
    sys.exit("macOS에서는 /dev/cu 포트로 보내면 신호가 쌓였다가 나중에 한꺼번에 전달됩니다. "
             "python -m tools.bt_remote 를 쓰세요.")
    port = args.port or default_port()
    if not port:
        sys.exit("BT-210 포트를 찾지 못했습니다. Mac 블루투스 설정에서 BT-210과 짝을 맺었는지 확인하세요.")
    remote = Remote(port, connect_wait=args.connect_wait)
    try:
        if args.cmd == "tap":
            remote.send(code, times=3)   # a short press: a few packets, like a real button
            remote.release()
        else:
            remote.hold(code, args.seconds)
        print(f"보냄: {args.combo} (코드 {code}) → {port}")
    finally:
        remote.close()


if __name__ == "__main__":
    main()
