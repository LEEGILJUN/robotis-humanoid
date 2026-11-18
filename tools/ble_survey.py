"""Scan, then connect to every unnamed BLE device and print its services (run with Bluetooth permission).

Helps pick out a module such as the BT-410 among phones/tablets that rotate random addresses.

    .venv/bin/python -m tools.ble_survey
"""
from __future__ import annotations

import asyncio

from bleak import BleakClient, BleakScanner


async def services_of(address: str) -> list[str]:
    try:
        async with BleakClient(address, timeout=10.0) as client:
            out = []
            for s in client.services:
                chars = ",".join(f"{c.uuid[4:8]}[{'/'.join(c.properties)}]" for c in s.characteristics)
                out.append(f"{s.uuid}  {s.description}  {chars}")
            return out
    except Exception as e:
        return [f"연결 실패: {type(e).__name__} {e}"]


async def main() -> None:
    found = await BleakScanner.discover(timeout=10.0, return_adv=True)
    rows = sorted(found.values(), key=lambda da: -da[1].rssi)
    for d, adv in rows:
        name = d.name or adv.local_name or ""
        mfr = ",".join(f"{k:04x}" for k in adv.manufacturer_data)  # 004c = Apple
        print(f"{adv.rssi:4d} dBm  {d.address}  {name!r}  mfr={mfr or '-'}  uuids={adv.service_uuids}")
        if not name and "004c" not in mfr:
            for line in await services_of(d.address):
                print(f"        {line}")


if __name__ == "__main__":
    asyncio.run(main())
