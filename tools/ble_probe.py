"""Find the robot's BT-410 over Bluetooth LE and show its services.

macOS only lets apps with Bluetooth permission use BLE, so run this from Terminal.app
(it asks for permission the first time), not from inside the Claude app.

    .venv/bin/python -m tools.ble_probe                # scan 12 s, list devices
    .venv/bin/python -m tools.ble_probe <address>      # connect, list services/characteristics
"""
from __future__ import annotations

import asyncio
import sys

from bleak import BleakClient, BleakScanner


async def scan() -> None:
    found = await BleakScanner.discover(timeout=12.0, return_adv=True)
    rows = sorted(found.values(), key=lambda da: -da[1].rssi)
    print(f"{len(rows)}개 장치 발견 (신호 강한 순)")
    for d, adv in rows:
        name = d.name or adv.local_name or ""
        mark = "  <-- ROBOTIS?" if any(k in name.upper() for k in ("BT", "ROBOTIS", "410")) else ""
        print(f"{adv.rssi:4d} dBm  {d.address}  {name!r}{mark}")


async def inspect(address: str) -> None:
    async with BleakClient(address, timeout=15.0) as client:
        print(f"연결됨: {address}")
        for service in client.services:
            print(f"service {service.uuid}  {service.description}")
            for ch in service.characteristics:
                print(f"   char {ch.uuid}  props={','.join(ch.properties)}  {ch.description}")


if __name__ == "__main__":
    asyncio.run(inspect(sys.argv[1]) if len(sys.argv) > 1 else scan())
