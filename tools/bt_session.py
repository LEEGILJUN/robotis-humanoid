"""Keep one Bluetooth connection to the BT-210 open and send remote commands on demand.

Connecting takes 3-25 s on this Mac, and connecting per command made it impossible to check
the link before sending. This keeps the RFCOMM channel open, prints the link state every few
seconds, and executes commands as they arrive, either typed in this terminal or written as
lines to the command file (so another process can drive it):

    .venv/bin/python -m tools.bt_session                 # in a terminal with Bluetooth permission
    echo "tap 1+2" >> logs/bt_cmd                        # from anywhere

Commands: tap <combo> | hold <combo> <seconds> | stop | status | quit
Safety: "no button" is sent on connect, after every command and on exit; holds are capped at 3 s.
"""
from __future__ import annotations

import os
import select
import sys
import time
from pathlib import Path

from tools.bt_remote import BTRemote, pump
from tools.rc100 import code_of

CMD_FILE = Path(__file__).resolve().parent.parent / "logs" / "bt_cmd"


def link_state(remote: BTRemote) -> str:
    ch = remote.channel
    open_ = bool(ch is not None and ch.isOpen())
    return f"device_connected={remote.dev.isConnected()} channel_open={open_}"


def execute(remote: BTRemote, line: str) -> bool:
    parts = line.split()
    if not parts:
        return True
    cmd = parts[0].lower()
    try:
        if cmd == "quit":
            return False
        if cmd == "status":
            print(f"[상태] {link_state(remote)}")
        elif cmd == "stop":
            remote.release()
            print("[보냄] 정지(버튼 없음)")
        elif cmd == "tap" and len(parts) == 2:
            if not remote.channel.isOpen():
                print("[거부] 채널이 닫혀 있어 보내지 않았습니다.")
                return True
            remote.tap(code_of(parts[1]))
            print(f"[보냄] tap {parts[1]} (코드 {code_of(parts[1])})")
        elif cmd == "hold" and len(parts) == 3:
            if not remote.channel.isOpen():
                print("[거부] 채널이 닫혀 있어 보내지 않았습니다.")
                return True
            remote.hold(code_of(parts[1]), float(parts[2]))
            print(f"[보냄] hold {parts[1]} {parts[2]}s")
        else:
            print(f"[알 수 없는 명령] {line!r}")
    except Exception as e:
        print(f"[오류] {e}")
        remote.release()
    sys.stdout.flush()
    return True


def main() -> None:
    CMD_FILE.parent.mkdir(exist_ok=True)
    CMD_FILE.write_text("")  # start empty: never replay old commands
    remote = BTRemote()
    remote.info()
    remote.open()  # sends "no button" right after connecting
    print(f"[연결 유지 중] {link_state(remote)}")
    print("BT-210의 파란 LED가 계속 켜져 있으면 연결된 것입니다. 명령을 입력하거나 logs/bt_cmd 에 쓰세요.")
    sys.stdout.flush()
    offset = 0
    last_status = time.monotonic()
    try:
        while True:
            pump(0.05)
            # commands typed here
            if select.select([sys.stdin], [], [], 0)[0]:
                if not execute(remote, sys.stdin.readline().strip()):
                    break
            # commands written to the file
            size = os.path.getsize(CMD_FILE)
            if size > offset:
                with open(CMD_FILE, encoding="utf-8") as f:
                    f.seek(offset)
                    new = f.read()
                offset = size
                for line in new.splitlines():
                    print(f"> {line}")
                    if not execute(remote, line.strip()):
                        raise KeyboardInterrupt
            if time.monotonic() - last_status > 5:
                print(f"[상태] {link_state(remote)}")
                sys.stdout.flush()
                last_status = time.monotonic()
                if not remote.channel.isOpen():
                    print("[경고] 연결이 끊어졌습니다. 세션을 다시 시작하세요.")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        print("세션 종료: 정지 신호를 보내고 연결을 닫습니다.")
        remote.close()


if __name__ == "__main__":
    main()
