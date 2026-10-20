"""
加速度を OSC で送る（初期設定は 127.0.0.1:5005、アドレス /acc）

    pip install python-osc

切断したら再接続する。受信側の例は ORPHE-CORE.py の wiki
「OSCで様々なアプリケーションにデータをリアルタイム送信する」を参照。
"""

import asyncio

from pythonosc import udp_client

from orphe_core_insole import OrpheCoreInsole

ip = "127.0.0.1"
port = 5005
client = udp_client.SimpleUDPClient(ip, port)


async def start(device):
    if device.kind == 'core':
        await device.set_led(1, 0)
        await device.set_acc_range(16)
    else:
        await device.set_data_streaming_mode(4)
    await device.start_sensor_values_notification()


async def main():
    device = OrpheCoreInsole()
    device.on('converted_acc', lambda acc: client.send_message("/acc", [acc.x, acc.y, acc.z]))

    if not await device.connect():
        return
    await start(device)

    try:
        while True:
            await asyncio.sleep(1)
            if not device.is_connected():
                print('Reconnecting...')
                if not await device.connect():
                    break
                await start(device)
    finally:
        if device.is_connected():
            await device.stop_sensor_values_notification()
            await device.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("KeyboardInterrupt(Ctrl+C) received.")
