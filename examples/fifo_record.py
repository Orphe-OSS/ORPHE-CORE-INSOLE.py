"""
FIFO（ところてん）で欠損なく収録して CSV に保存する

CORE は FW 20260905 以降、INSOLE は FW 20260510 以降が必要。

    python examples/fifo_record.py 30 capture.csv   # 30 秒収録
"""

import asyncio
import sys

from orphe_core_insole import FifoRecorder, OrpheCoreInsole


async def main(seconds: float, path: str):
    device = OrpheCoreInsole()
    if not await device.connect():
        return

    fifo = FifoRecorder(device)
    fifo.on('progress', lambda info: print(f"collected {info.collected} lag {info.lag} dropped {info.dropped}"))
    fifo.on('data_loss', lambda info: print(f"欠損: {info.reason} {info.dropped}"))
    fifo.on('error', lambda error: print(f"エラー: {error}"))
    try:
        if not await fifo.start():
            return
        await asyncio.sleep(seconds)
    finally:
        await fifo.stop()  # 取りこぼしを回収してからリアルタイム通信に戻す
        await device.disconnect()

    fifo.save_csv(path)
    print(f"{fifo.collected_count} packets, dropped {fifo.dropped_count} → {path}")


if __name__ == "__main__":
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10
    path = sys.argv[2] if len(sys.argv) > 2 else 'fifo.csv'
    asyncio.run(main(seconds, path))
