"""LED を 1 秒ごとに点滅させる（ORPHE CORE）。終了は Ctrl+C"""

import asyncio

from orphe_core_insole import OrpheCoreInsole, core_profile


async def main():
    device = OrpheCoreInsole(core_profile())
    if not await device.connect():
        return

    await device.print_device_information()
    await device.set_led_brightness(255)
    try:
        while True:
            await device.set_led(1, 0)
            await asyncio.sleep(1)
            await device.set_led(0, 0)
            await asyncio.sleep(1)
    finally:
        print("Disconnecting from the device.")
        await device.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("KeyboardInterrupt(Ctrl+C) received.")
