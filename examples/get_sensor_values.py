"""
センサー値（加速度・角速度・クォータニオン・圧力）を表示する

CORE でも INSOLE でも動く。終了は Ctrl+C。
注意）CORE の 50Hz モデルではシリアル番号とパケット番号は取得できない。
"""

import asyncio

from orphe_core_insole import OrpheCoreInsole


def lost_data(serial_number_prev, serial_number):
    print(f"Data loss detected. {serial_number_prev} <-> {serial_number}")


async def main():
    device = OrpheCoreInsole()

    # 見たいイベントを購読する（acc / gyro は raw / 32768、converted_* は [G] と [deg/s]）
    device.on('converted_acc', lambda acc: acc.print())
    # device.on('converted_gyro', lambda gyro: gyro.print())
    # device.on('quat', lambda quat: quat.print())
    device.on('pressure', lambda pressure: pressure.print())  # INSOLE のみ
    device.on('lost_data', lost_data)
    device.on('disconnect', lambda: print("Disconnected from the device."))

    if not await device.connect():
        return

    if device.kind == 'core':
        await device.set_led(1, 0)
        await device.set_acc_range(16)
    else:
        # 1: quat・gyro・acc 200Hz / 3: gyro・acc・圧力 200Hz / 4: quat・gyro・acc・圧力 100Hz
        await device.set_data_streaming_mode(4)
    await device.start_sensor_values_notification()

    try:
        while device.is_connected():
            await asyncio.sleep(1)
    finally:
        if device.is_connected():
            print("Stopping notification...")
            await device.stop_sensor_values_notification()
            await device.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("KeyboardInterrupt(Ctrl+C) received.")
