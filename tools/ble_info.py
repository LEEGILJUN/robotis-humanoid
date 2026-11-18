"""Read the standard Device Information strings of a BLE device (run from a terminal with Bluetooth permission).

    .venv/bin/python -m tools.ble_info <address> [<address> ...]
"""
from __future__ import annotations

import asyncio
import sys

from bleak import BleakClient

CHARS = {
    "00002a29-0000-1000-8000-00805f9b34fb": "manufacturer",
    "00002a24-0000-1000-8000-00805f9b34fb": "model",
    "00002a00-0000-1000-8000-00805f9b34fb": "device name",
}


async def info(address: str) -> None:
    try:
        async with BleakClient(address, timeout=15.0) as client:
            print(f"{address}")
            found = False
            for service in client.services:
                for ch in service.characteristics:
                    if ch.uuid in CHARS and "read" in ch.properties:
                        try:
                            value = await client.read_gatt_char(ch)
                            print(f"   {CHARS[ch.uuid]}: {value.decode(errors='replace')!r}")
                            found = True
                        except Exception as e:  # some reads need pairing
                            print(f"   {CHARS[ch.uuid]}: (읽기 실패: {e})")
            if not found:
                print("   (Device Information 없음)")
    except Exception as e:
        print(f"{address}: 연결 실패 ({e})")


async def main(addresses: list[str]) -> None:
    for a in addresses:
        await info(a)


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1:]))
