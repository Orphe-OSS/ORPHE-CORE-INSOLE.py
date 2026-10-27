"""
matplotlib でセンサー値をリアルタイムに描く

    pip install matplotlib
    python examples/plot_sensor_values.py            # 加速度（CORE / INSOLE）
    python examples/plot_sensor_values.py pressure   # 圧力 6ch（INSOLE）
"""

import asyncio
import sys
from collections import deque

import matplotlib.pyplot as plt
from matplotlib import animation

from orphe_core_insole import OrpheCoreInsole

plot_buffer_size = 512  # バッファサイズ
update_interval = 0.02  # 描画更新間隔（秒）

show_pressure = len(sys.argv) > 1 and sys.argv[1] == 'pressure'
labels = [f'Sensor {i + 1}' for i in range(6)] if show_pressure else ['Acc X', 'Acc Y', 'Acc Z']
series = [deque(maxlen=plot_buffer_size) for _ in labels]

fig, ax = plt.subplots()
lines = [ax.plot([], [], label=label)[0] for label in labels]
ax.legend()
ax.set_xlim(0, plot_buffer_size)
ax.set_ylim(*((0, 1024) if show_pressure else (-3, 3)))  # 適宜調整


def update(frame):
    for line, values in zip(lines, series):
        line.set_data(range(len(values)), values)
    return lines


def append(values):
    for buffer, value in zip(series, values):
        buffer.append(value)


async def main():
    device = OrpheCoreInsole()
    if show_pressure:
        device.on('pressure', lambda pressure: append(pressure.values))
    else:
        device.on('converted_acc', lambda acc: append((acc.x, acc.y, acc.z)))
    device.on('lost_data', lambda prev, cur: print(f"Data loss detected. {prev} <-> {cur}"))

    if not await device.connect():
        return
    if device.kind == 'core':
        await device.set_led(1, 0)
        await device.set_acc_range(16)
    else:
        await device.set_data_streaming_mode(4)
    await device.start_sensor_values_notification()

    # FuncAnimation は変数に持っておかないと描画されない
    _animation = animation.FuncAnimation(fig, update, interval=20, blit=True)
    # ほかのコールバック処理を止めないよう、ブロックせずにウィンドウを出す
    plt.show(block=False)

    try:
        while device.is_connected() and plt.fignum_exists(fig.number):
            plt.pause(update_interval)
            await asyncio.sleep(update_interval)
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
