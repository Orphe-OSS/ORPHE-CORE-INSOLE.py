"""device information（バッテリー・レンジなど）を表示する"""

import asyncio

from orphe_core_insole import OrpheCoreInsole


async def main():
    device = OrpheCoreInsole()

    # 特定のデバイスにつなぐときは connect() にアドレスを渡す（アドレスは scan.py で確認できる）
    if not await device.connect():
        return

    try:
        print("Initial state:")
        await device.print_device_information()

        # 設定できる項目（必要なものだけコメントを外す）
        # await device.set_acc_range(16)      # 2, 4, 8, 16[G]
        # await device.set_gyro_range(2000)   # 250, 500, 1000, 2000[deg/s]
        # if device.kind == 'core':
        #     await device.set_led(1, 1)
        #     await device.set_led_brightness(255)
        #     await device.set_lr(0)
        # else:
        #     await device.set_mount_position(1)  # 0: 足底・左, 1: 足底・右, 2: 足背・左, 3: 足背・右

        print("\nAfter state:")
        await device.print_device_information()
    finally:
        print("Disconnecting from the device.")
        await device.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
