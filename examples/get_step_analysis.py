"""
歩行解析（STEP_ANALYSIS）の値を表示する（ORPHE CORE）

終了は Ctrl+C。
"""

import asyncio

from orphe_core_insole import OrpheCoreInsole, core_profile


async def main():
    device = OrpheCoreInsole(core_profile())
    device.on('gait', lambda gait: gait.print())
    device.on('stride', lambda stride: stride.print())
    device.on('pronation', lambda pronation: pronation.print())
    # 歩いていない間もたくさん届くので、見やすさのためコメントアウトしている
    # device.on('quat_distance', lambda quat_distance: quat_distance.print())

    if not await device.connect():
        return

    await device.print_device_information()
    await device.start_step_analysis_notification()

    try:
        while device.is_connected():
            await asyncio.sleep(1)
    finally:
        if device.is_connected():
            print("Stopping notification...")
            await device.stop_step_analysis_notification()
            await device.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("KeyboardInterrupt(Ctrl+C) received.")
