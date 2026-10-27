"""周りの BLE デバイスをスキャンして、ORPHE CORE / INSOLE のアドレスを確認する"""

import asyncio

from orphe_core_insole import OrpheCoreInsole


async def main():
    device = OrpheCoreInsole()
    # all_devices=False にすると ORPHE のデバイスだけを表示する
    for result in await device.scan(all_devices=True):
        kind = result.kind or "-"
        print(f"Device: {result.name}(name), {result.address}(address), {result.rssi}(rssi), {kind}(kind)")


if __name__ == "__main__":
    asyncio.run(main())
