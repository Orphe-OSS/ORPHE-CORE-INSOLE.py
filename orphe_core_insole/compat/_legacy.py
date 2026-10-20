"""
ORPHE-CORE.py / ORPHE-INSOLE.py 互換の共通部分

公開版の `Orphe` は `set_got_acc_callback()` などでコールバックを 1 つずつ持ち、
値の範囲外は例外ではなくメッセージを表示して何もしない。その振る舞いを
OrpheCoreInsole の上に載せる。
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from ..data import Range
from ..device import OrpheCoreInsole
from ..packets import parse_step_analysis
from ..profiles.base import DeviceProfile

# イベント名 -> 公開版のコールバック属性名
LEGACY_CALLBACK_NAMES = {
    'acc': 'got_acc_callback',
    'converted_acc': 'got_converted_acc_callback',
    'gyro': 'got_gyro_callback',
    'converted_gyro': 'got_converted_gyro_callback',
    'pressure': 'got_pressure_callback',
    'quat': 'got_quat_callback',
    'gait': 'got_gait_callback',
    'stride': 'got_stride_callback',
    'pronation': 'got_pronation_callback',
    'quat_distance': 'got_quat_distance_callback',
    'lost_data': 'lost_data_callback',
    'disconnect': 'on_disconnect_callback',
}


def dispatch_legacy(owner: Any, event: str, *args: Any) -> None:
    """owner にコールバック属性があれば呼ぶ（公開版の `hasattr(owner, ...)` と同じ判定）"""
    name = LEGACY_CALLBACK_NAMES[event]
    if hasattr(owner, name) and getattr(owner, name):
        getattr(owner, name)(*args)


def make_sensor_values_data(profile: DeviceProfile) -> type:
    """公開版の SensorValuesData(owner, data, sensor_range) と同じ使い方のクラスを作る"""

    class SensorValuesData:
        """
        センサーの値を格納するクラス（公開版互換）

        パースしたサンプルごとに owner の got_*_callback を呼ぶ。属性には最後のサンプルが残る。
        """

        def __init__(self, owner: Any, data: bytes | bytearray, sensor_range: Range) -> None:
            packet = profile.parse_sensor_values(data, sensor_range)
            if packet is None:
                return
            self.data = data
            self.type = packet.header
            if packet.serial_number is not None:
                self.serial_number = packet.serial_number
            self.timestamp = packet.timestamp
            for frame in packet.frames:
                self.acc = frame.acc
                self.converted_acc = frame.converted_acc
                self.gyro = frame.gyro
                self.converted_gyro = frame.converted_gyro
                if frame.quat is not None:
                    self.quat = frame.quat
                if frame.pressure is not None:
                    self.pressure = frame.pressure
                dispatch_legacy(owner, 'acc', frame.acc)
                dispatch_legacy(owner, 'converted_acc', frame.converted_acc)
                dispatch_legacy(owner, 'gyro', frame.gyro)
                dispatch_legacy(owner, 'converted_gyro', frame.converted_gyro)
                if frame.pressure is not None:
                    dispatch_legacy(owner, 'pressure', frame.pressure)
                if frame.quat is not None:
                    dispatch_legacy(owner, 'quat', frame.quat)

    return SensorValuesData


class StepAnalysisData:
    """
    ステップ解析の値を格納するクラス（公開版互換）

    owner.step_count を更新し、歩数が増えたときだけ owner の got_*_callback を呼ぶ。
    """

    def __init__(self, owner: Any, data: bytes | bytearray) -> None:
        self.data = data
        if data[1] in (0, 1, 2, 4):
            self.step_count = int.from_bytes(data[2:4], byteorder='big', signed=False)
        result = parse_step_analysis(data, owner.step_count)
        if result is None:
            return
        event, value = result
        setattr(self, event, value)
        dispatch_legacy(owner, event, value)


class LegacyOrphe(OrpheCoreInsole):
    """公開版の Orphe クラスと同じメソッドを持つ OrpheCoreInsole"""

    def __init__(self, profile: DeviceProfile, **kwargs: Any) -> None:
        super().__init__(profile, **kwargs)
        # 公開版は owner の属性を見てコールバックを呼ぶので、各イベントを属性へ中継する
        for event in LEGACY_CALLBACK_NAMES:
            self.on(event, self._legacy_dispatcher(event))

    def _legacy_dispatcher(self, event: str) -> Callable[..., None]:
        return lambda *args: dispatch_legacy(self, event, *args)

    # ── コールバックの設定 ────────────────────────────────────

    def set_lost_data_callback(self, callback: Callable[[int, int], Any]) -> None:
        """データが欠損したときに呼び出されるコールバック関数を設定する"""
        self.lost_data_callback = callback

    def set_got_acc_callback(self, callback: Callable[[Any], Any]) -> None:
        """加速度センサの値（raw / 32768）を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_acc_callback = callback

    def set_got_gyro_callback(self, callback: Callable[[Any], Any]) -> None:
        """ジャイロセンサの値（raw / 32768）を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_gyro_callback = callback

    def set_got_converted_acc_callback(self, callback: Callable[[Any], Any]) -> None:
        """加速度[G]を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_converted_acc_callback = callback

    def set_got_converted_gyro_callback(self, callback: Callable[[Any], Any]) -> None:
        """角速度[deg/s]を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_converted_gyro_callback = callback

    def set_got_quat_callback(self, callback: Callable[[Any], Any]) -> None:
        """クォータニオンの値を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_quat_callback = callback

    def set_got_gait_callback(self, callback: Callable[[Any], Any]) -> None:
        """歩行解析の値を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_gait_callback = callback

    def set_got_stride_callback(self, callback: Callable[[Any], Any]) -> None:
        """ストライドの値を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_stride_callback = callback

    def set_got_pronation_callback(self, callback: Callable[[Any], Any]) -> None:
        """プロネーションの値を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_pronation_callback = callback

    def set_got_quat_distance_callback(self, callback: Callable[[Any], Any]) -> None:
        """クォータニオンと差分値を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_quat_distance_callback = callback

    def set_on_disconnect_callback(self, callback: Callable[[], Any]) -> None:
        """接続が切断されたときに呼び出されるコールバック関数を設定する"""
        self.on_disconnect_callback = callback

    async def disconnect_callback(self, owner: Any) -> None:
        dispatch_legacy(owner, 'disconnect')

    # ── 値の範囲外はメッセージを表示して何もしない ────────────

    async def set_acc_range(self, acc_range: int) -> None:
        try:
            await super().set_acc_range(acc_range)
        except ValueError as error:
            print(error)

    async def set_gyro_range(self, gyro_range: int) -> None:
        try:
            await super().set_gyro_range(gyro_range)
        except ValueError as error:
            print(error)
