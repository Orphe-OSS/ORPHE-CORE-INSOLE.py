"""
INSOLE の歩行解析を 1 歩ずつ表示し、CSV に保存する

    python examples/get_insole_gait.py 60 gait.csv   # 60 秒（Ctrl+C でも終了）
"""

import asyncio
import sys

from orphe_core_insole import InsoleGait, OrpheCoreInsole, insole_profile


async def main(seconds: float, path: str):
    device = OrpheCoreInsole(insole_profile())
    if not await device.connect():
        return

    gait = InsoleGait(device)
    gait.on('gait', lambda row: print(
        f"step {row.step_number}: {row.gait_type} stride {row.stride_norm_m} m, "
        f"cadence {row.cadence_hz} Hz, {row.foot_strike}, {row.pronation_type}"))
    gait.on('error', lambda error: print(f"エラー: {error}"))
    try:
        if not await gait.start():
            return
        await asyncio.sleep(seconds)
    finally:
        await gait.stop()
        await device.disconnect()

    gait.save_csv(path)
    print(f"{gait.step_count} steps → {path}")


if __name__ == "__main__":
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30
    path = sys.argv[2] if len(sys.argv) > 2 else 'gait.csv'
    asyncio.run(main(seconds, path))
