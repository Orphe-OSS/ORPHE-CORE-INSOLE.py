"""デバイスごとの差分を表すプロファイルの共通インターフェース"""

from __future__ import annotations

from typing import Literal, Protocol

from ..data import Range
from ..packets import SensorValuesPacket

DeviceKind = Literal['core', 'insole']

# センサレンジのテーブル（device information の range.acc / range.gyro のインデックス）
ACC_RANGES = [2, 4, 8, 16]           # [G]
GYRO_RANGES = [250, 500, 1000, 2000]  # [deg/s]

# レンジ設定を取得できないとき・範囲外のときのレンジ
DEFAULT_ACC_RANGE = 16     # [G]
DEFAULT_GYRO_RANGE = 2000  # [deg/s]

# ジャイロの実感度[dps/LSB]。IMU(LSM6DSOX)のデータシート代表感度は
# ±250/500/1000/2000 dps でそれぞれ 8.75/17.5/35/70 mdps/LSB であり、
# これは「フルスケール[dps] × 0.000035」に等しい。
GYRO_DPS_PER_LSB_PER_RANGE = 0.000035


class DeviceInformation(Protocol):
    """device information に共通する項目"""

    data: bytes | bytearray
    battery: int
    range: Range


class DeviceProfile(Protocol):
    """
    CORE / INSOLE の差分

    通信（スキャン・接続・notify・書き込み）は OrpheCoreInsole が共通で持ち、
    パケットの中身とコマンドのバイト列だけをプロファイルが決める。
    """

    kind: DeviceKind
    # ログに出すデバイス名
    label: str
    # device information を書き込んだあと読み直せるようになるまでの待ち時間[s]
    write_wait_sec: float
    # データ欠損（シリアル番号の飛び）を判定する SENSOR_VALUES のヘッダ
    lost_data_headers: frozenset[int]

    def matches_name(self, name: str | None) -> bool:
        """スキャン結果のデバイス名がこのデバイスか"""
        ...

    def parse_device_information(self, data: bytes | bytearray) -> DeviceInformation:
        ...

    def describe_device_information(self, di: DeviceInformation) -> list[str]:
        """print_device_information() で表示する行"""
        ...

    def build_range_write(self, di: DeviceInformation) -> bytearray:
        """di のレンジ設定を書き込むバイト列"""
        ...

    def parse_sensor_values(self, data: bytes | bytearray, sensor_range: Range) -> SensorValuesPacket | None:
        """扱わないヘッダなら None"""
        ...


def acc_range_index(acc_range: int) -> int:
    """2, 4, 8, 16[G] を device information のインデックスにする"""
    if acc_range not in ACC_RANGES:
        raise ValueError("acc_range must be 2, 4, 8, or 16[g].")
    return ACC_RANGES.index(acc_range)


def gyro_range_index(gyro_range: int) -> int:
    """250, 500, 1000, 2000[deg/s] を device information のインデックスにする"""
    if gyro_range not in GYRO_RANGES:
        raise ValueError("gyro_range must be 250, 500, 1000, or 2000[deg/s].")
    return GYRO_RANGES.index(gyro_range)


def _range_from_setting(ranges: list[int], setting: object, fallback: int) -> int:
    # setting は device information のインデックスのみ受け付ける（範囲外・非整数は fallback）
    is_index = isinstance(setting, int) and not isinstance(setting, bool) and 0 <= setting < len(ranges)
    return ranges[setting] if is_index else fallback  # type: ignore[index]


def imu_range_from_settings(sensor_range: Range | None) -> tuple[int, int]:
    """device information のレンジ設定（インデックス）を (加速度[G], 角速度[deg/s]) にする。未取得・範囲外は 16G / 2000dps"""
    return (
        _range_from_setting(ACC_RANGES, getattr(sensor_range, 'acc', None), DEFAULT_ACC_RANGE),
        _range_from_setting(GYRO_RANGES, getattr(sensor_range, 'gyro', None), DEFAULT_GYRO_RANGE),
    )


def gyro_raw_to_dps(raw: float, gyro_range: int) -> float:
    """ジャイロの int16 生値 → deg/s。gyro_range はフルスケール[deg/s]"""
    return raw * gyro_range * GYRO_DPS_PER_LSB_PER_RANGE


def i16(data: bytes | bytearray, at: int) -> int:
    """符号付き 16 ビット（ビッグエンディアン）"""
    return int.from_bytes(data[at:at + 2], byteorder='big', signed=True)


def u16(data: bytes | bytearray, at: int) -> int:
    """符号なし 16 ビット（ビッグエンディアン）"""
    return int.from_bytes(data[at:at + 2], byteorder='big', signed=False)
