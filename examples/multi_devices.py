"""
2 台同時に接続してセンサー値の欠損を監視する

CORE 2 台・INSOLE 2 台・CORE と INSOLE の組み合わせのどれでも動く。
接続中のデバイスはアドバタイズを止めるので、2 台目の connect() は別のデバイスにつながる。
"""

import asyncio

from orphe_core_insole import OrpheCoreInsole


async def main():
    devices = [OrpheCoreInsole(), OrpheCoreInsole()]
    for number, device in enumerate(devices, start=1):
        device.on('lost_data', lambda prev, cur, n=number: print(f"[{n}] Data loss detected. {prev} <-> {cur}"))

    for device in devices:
        if not await device.connect():
            return
        await device.print_device_information()
        if device.kind == 'insole':
            await device.set_data_streaming_mode(4)
        else:
            await device.set_led_brightness(255)
        await device.start_sensor_values_notification()

    try:
        while True:
            await asyncio.sleep(1)
    finally:
        print("Disconnecting from the devices.")
        for device in devices:
            if device.is_connected():
                await device.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("KeyboardInterrupt(Ctrl+C) received.")
