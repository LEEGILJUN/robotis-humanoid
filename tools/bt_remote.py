"""RC-100 remote over the BT-210 using macOS IOBluetooth directly (RFCOMM), not /dev/cu.*.

Opening /dev/cu.ROBOTISBT-210 does not bring the radio link up on this Mac (the BT-210 LED keeps
blinking and nothing reaches the CM-530), so this opens the RFCOMM channel itself, the way
R+ Task 2.0 does. Needs Bluetooth permission: run it from a terminal, not from the Claude app.

    .venv/bin/python -m tools.bt_remote info
    .venv/bin/python -m tools.bt_remote tap 2+U
    .venv/bin/python -m tools.bt_remote hold U 1.5
    .venv/bin/python -m tools.bt_remote stream 2+U 2       # repeat (max 3 s)

Safety: every session first sends "no button" five times, holds are capped at 3 s, and the
session always ends with "no button", also on Ctrl-C.
"""
from __future__ import annotations

import argparse
import sys
import time

import objc
from Foundation import NSDate, NSObject, NSRunLoop
from IOBluetooth import IOBluetoothDevice, IOBluetoothSDPUUID

from tools.rc100 import code_of, packet

DEFAULT_ADDRESS = "b8-63-bc-00-6a-10"   # sticker on the BT-210: B863BC006A10
SPP_UUID16 = 0x1101


def pump(seconds: float) -> None:
    NSRunLoop.currentRunLoop().runUntilDate_(NSDate.dateWithTimeIntervalSinceNow_(seconds))


class ChannelDelegate(NSObject):
    def init(self):
        self = objc.super(ChannelDelegate, self).init()
        self.rx = bytearray()
        self.opened = None
        return self

    def rfcommChannelData_data_length_(self, channel, data, length):
        try:
            self.rx += bytes(data[:length])
        except Exception:
            pass

    def rfcommChannelOpenComplete_status_(self, channel, status):
        self.opened = status

    def rfcommChannelClosed_(self, channel):
        print("채널 닫힘")


class BTRemote:
    def __init__(self, address: str = DEFAULT_ADDRESS):
        self.dev = IOBluetoothDevice.deviceWithAddressString_(address)
        if self.dev is None:
            sys.exit(f"장치를 찾을 수 없습니다: {address}")
        self.delegate = ChannelDelegate.alloc().init()
        self.channel = None

    def info(self) -> None:
        d = self.dev
        print(f"name={d.name()} paired={d.isPaired()} connected={d.isConnected()} address={d.addressString()}")

    def channel_id(self) -> int:
        rec = self.dev.getServiceRecordForUUID_(IOBluetoothSDPUUID.uuid16_(SPP_UUID16))
        if rec is None:
            print("SDP 정보를 새로 조회합니다...")
            self.dev.performSDPQuery_(None)
            pump(6.0)
            rec = self.dev.getServiceRecordForUUID_(IOBluetoothSDPUUID.uuid16_(SPP_UUID16))
        if rec is None:
            print("SPP 서비스 기록이 없어 채널 1을 시도합니다.")
            return 1
        err, cid = rec.getRFCOMMChannelID_(None)
        print(f"SPP RFCOMM 채널 = {cid} (err={err})")
        return cid if err == 0 else 1

    def open(self) -> None:
        t0 = time.monotonic()
        cid = self.channel_id()
        if not self.dev.isConnected():
            err = self.dev.openConnection()  # baseband link first; RFCOMM on top of it
            print(f"기본 연결: err={err & 0xFFFFFFFF:#x}, connected={self.dev.isConnected()}")
            pump(1.0)
        err, channel = -1, None
        for attempt in range(1, 4):
            result = self.dev.openRFCOMMChannelSync_withChannelID_delegate_(None, cid, self.delegate)
            err, channel = (result if isinstance(result, tuple) else (result, None))
            if err == 0 and channel is not None:
                break
            print(f"RFCOMM 시도 {attempt} 실패: err={err & 0xFFFFFFFF:#x}")
            pump(2.0)
        if err != 0 or channel is None:
            sys.exit("RFCOMM 채널을 열지 못했습니다.")
        self.channel = channel
        print(f"연결됨: 채널 {cid}, {time.monotonic() - t0:.1f}초, MTU {channel.getMTU()}")
        # Safety: start every session with "no button" so nothing left over keeps the robot moving.
        for _ in range(5):
            self.send(0)
            pump(0.05)
        pump(0.5)

    def send(self, code: int) -> None:
        data = packet(code)
        err = self.channel.writeSync_length_(data, len(data))
        if err != 0:
            print(f"쓰기 실패 err={err:#x}")
        pump(0.01)

    def tap(self, code: int) -> None:
        for _ in range(3):
            self.send(code)
            pump(0.05)
        self.release()

    MAX_HOLD_S = 3.0

    def hold(self, code: int, seconds: float) -> None:
        if seconds > self.MAX_HOLD_S:
            print(f"안전을 위해 {self.MAX_HOLD_S}초로 줄입니다 (요청 {seconds}초).")
            seconds = self.MAX_HOLD_S
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(code)
            pump(0.05)
        self.release()

    def release(self) -> None:
        for _ in range(3):
            self.send(0)
            pump(0.05)

    def close(self) -> None:
        if self.channel is not None:
            try:
                self.release()  # always leave the robot with "no button"
            except Exception:
                pass
        pump(0.5)
        if self.channel is not None:
            self.channel.closeChannel()
        self.dev.closeConnection()
        if self.delegate.rx:
            print(f"받은 데이터 {len(self.delegate.rx)}바이트: {bytes(self.delegate.rx[:32]).hex(' ')}")


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--address", default=DEFAULT_ADDRESS)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("tap").add_argument("combo")
    h = sub.add_parser("hold")
    h.add_argument("combo")
    h.add_argument("seconds", type=float)
    s = sub.add_parser("stream")
    s.add_argument("combo")
    s.add_argument("seconds", type=float)
    args = ap.parse_args(argv)

    remote = BTRemote(args.address)
    remote.info()
    if args.cmd == "info":
        return
    code = code_of(args.combo)
    remote.open()
    try:
        if args.cmd == "tap":
            remote.tap(code)
        else:
            remote.hold(code, args.seconds)
        print(f"보냄: {args.combo} (코드 {code})")
    finally:
        remote.close()


if __name__ == "__main__":
    main()
