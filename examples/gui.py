"""
tkinter で 2 台の接続・切断を操作する GUI

電波状況による切断や手動切断、バッテリーレベル表示を実装している。
CORE / INSOLE はデバイス名で自動判別する。tkinter は Python 標準ライブラリなので追加のインストールは不要。
"""

import asyncio
import threading
from tkinter import DISABLED, NORMAL, Button, Label, LabelFrame, Tk

from orphe_core_insole import OrpheCoreInsole

BATTERY_LEVELS = {0: 'LOW', 1: 'MID', 2: 'HIGH'}


class DevicePanel:
    """1 台ぶんの接続ボタン・状態表示"""

    def __init__(self, app, title):
        self.app = app
        self.title = title
        self.device = OrpheCoreInsole()
        self.device.on('lost_data', lambda prev, cur: print(f"[{title}] Data loss detected. {prev} <-> {cur}"))
        self.device.on('disconnect', self.on_disconnect)

        frame = LabelFrame(app.master, text=title)
        frame.pack(pady=10, padx=10, fill="both", expand="yes")
        self.connect_button = Button(frame, text="Connect", command=self.connect)
        self.connect_button.pack(pady=5)
        self.disconnect_button = Button(frame, text="Disconnect", command=self.disconnect, state=DISABLED)
        self.disconnect_button.pack(pady=5)
        self.battery_label = Label(frame, text="Battery Level: -")
        self.battery_label.pack(pady=5)
        self.status_label = Label(frame, text="Status: Disconnected")
        self.status_label.pack(pady=5)

    def ui(self, widget, **options):
        """イベントループのスレッドから tkinter を触るときは after で UI スレッドに渡す"""
        self.app.master.after(0, lambda: widget.config(**options))

    def connect(self):
        self.connect_button.config(state=DISABLED)
        self.status_label.config(text='Status: Connecting...')
        self.app.run(self.connect_task())

    async def connect_task(self):
        try:
            if not await self.device.connect():
                print(f"Failed to connect to {self.title}.")
                self.ui(self.connect_button, state=NORMAL)
                self.ui(self.status_label, text='Status: Failed to connect')
                return
            if self.device.kind == 'core':
                await self.device.set_led(1, 0)
                await self.device.set_acc_range(16)
            else:
                await self.device.set_data_streaming_mode(4)
            await self.device.start_sensor_values_notification()
            print(f"Connected to {self.title} ({self.device.kind}) and started notifications.")

            level = self.device.device_information.battery
            self.ui(self.battery_label, text=f'Battery Level: {level} ({BATTERY_LEVELS.get(level, "-")})')
            self.ui(self.disconnect_button, state=NORMAL)
            self.ui(self.status_label, text=f'Status: Connected ({self.device.kind})')
        except Exception as error:  # noqa: BLE001 (接続失敗の理由を問わずボタンを戻す)
            print(f"Exception during connecting to {self.title}: {error}")
            self.ui(self.connect_button, state=NORMAL)
            self.ui(self.status_label, text='Status: Connection error')

    def disconnect(self):
        self.disconnect_button.config(state=DISABLED)
        self.status_label.config(text='Status: Disconnecting...')
        self.app.run(self.disconnect_task())

    async def disconnect_task(self):
        if self.device.is_connected():
            await self.device.stop_sensor_values_notification()
            await self.device.disconnect()
            print(f"Disconnected from {self.title}.")
        # 表示は切断監視の disconnect イベント（on_disconnect）で戻す

    def on_disconnect(self):
        print(f"Disconnected from {self.title}.")
        self.ui(self.connect_button, state=NORMAL)
        self.ui(self.disconnect_button, state=DISABLED)
        self.ui(self.status_label, text='Status: Disconnected')
        self.ui(self.battery_label, text='Battery Level: -')


class OrpheApp:

    def __init__(self, master):
        self.master = master
        self.master.title("Orphe Connection")
        self.master.focus_force()
        self.center_window(300, 400)

        # BLE はイベントループを別スレッドで回す
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.start_loop, daemon=True)
        self.loop_thread.start()

        self.panels = [DevicePanel(self, "Device 1"), DevicePanel(self, "Device 2")]
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def center_window(self, width, height):
        x = int((self.master.winfo_screenwidth() / 2) - (width / 2))
        y = int((self.master.winfo_screenheight() / 2) - (height / 2))
        self.master.geometry(f"{width}x{height}+{x}+{y}")

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def run(self, coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, self.loop)

    def on_closing(self):
        try:
            self.run(self.shutdown()).result(timeout=5)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.loop_thread.join()
            self.master.destroy()

    async def shutdown(self):
        await asyncio.gather(*(panel.device.disconnect() for panel in self.panels if panel.device.is_connected()))


if __name__ == "__main__":
    root = Tk()
    app = OrpheApp(root)
    root.mainloop()
