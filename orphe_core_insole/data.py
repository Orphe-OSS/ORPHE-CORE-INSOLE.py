"""
コールバックに渡すデータクラス

ORPHE-CORE.py / ORPHE-INSOLE.py と同じクラス名・属性・値にしている。
既存のコールバック関数がそのまま動くように、公開版の癖（struct.unpack の戻り値が
タプルのままの属性など）も変えていない。
"""

from __future__ import annotations

import struct
from datetime import datetime


def to_timestamp(hours: int, minutes: int, seconds: int, ms_high: int, ms_low: int) -> int:
    """
    v3のデータについてくるタイムスタンプをUNIX時刻[ms]にする

    日付はPCの現在日付を使い、時・分・秒・ミリ秒をパケットの値で置き換える。

    Args:
        hours: 時
        minutes: 分
        seconds: 秒
        ms_high: ミリ秒の上位バイト
        ms_low: ミリ秒の下位バイト
    """
    milliseconds = (ms_high << 8) | ms_low
    # FW はローカル時刻を送るので、タイムゾーンなしの現在時刻に合わせる
    now = datetime.now()  # noqa: DTZ005
    dt = now.replace(hour=hours, minute=minutes, second=seconds, microsecond=milliseconds * 1000)
    return int(dt.timestamp() * 1000)


class AccData:
    """
    加速度センサの値

    Attributes:
        x: x軸の加速度
        y: y軸の加速度
        z: z軸の加速度
        timestamp: タイムスタンプ
        serial_number: シリアルナンバー
        packet_number: パケットナンバー
    """

    def __init__(self) -> None:
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0
        self.timestamp: int = 0
        self.serial_number: int = 0
        self.packet_number: int = 0

    def print(self) -> None:
        print(
            f"Acc[{self.serial_number}][{self.packet_number}][{self.timestamp}]: {self.x}, {self.y}, {self.z}")


class GyroData:
    """
    ジャイロセンサの値

    Attributes:
        x: x軸の角速度
        y: y軸の角速度
        z: z軸の角速度
        timestamp: タイムスタンプ
        serial_number: シリアルナンバー
        packet_number: パケットナンバー
    """

    def __init__(self) -> None:
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0
        self.timestamp: int = 0
        self.serial_number: int = 0
        self.packet_number: int = 0

    def print(self) -> None:
        print(
            f"Gyro[{self.serial_number}][{self.packet_number}][{self.timestamp}]: {self.x}, {self.y}, {self.z}")


class QuatData:
    """
    クォータニオンの値

    Attributes:
        w, x, y, z: クォータニオン
        timestamp: タイムスタンプ
        serial_number: シリアルナンバー
        packet_number: パケットナンバー
    """

    def __init__(self) -> None:
        self.w: float = 0
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0
        self.timestamp: int = 0
        self.serial_number: int = 0
        self.packet_number: int = 0

    def print(self) -> None:
        print(
            f"Quat[{self.serial_number}][{self.packet_number}][{self.timestamp}]: {self.w}, {self.x}, {self.y}, {self.z}")


class PressureData:
    """
    圧力センサの値（INSOLE のみ）

    Attributes:
        values: 6ch の圧力値（ADC の生値。converted_pressure では [N]）
        timestamp: タイムスタンプ
        serial_number: シリアルナンバー
        packet_number: パケットナンバー
    """

    def __init__(self) -> None:
        self.values: list[float] = []
        self.timestamp: int = 0
        self.serial_number: int = 0
        self.packet_number: int = 0

    def print(self) -> None:
        print(
            f"Pressure[{self.serial_number}][{self.packet_number}][{self.timestamp}]: {self.values}")


class Range:
    """
    加速度センサとジャイロセンサのレンジ（device information のインデックス 0-3）

    Attributes:
        acc: 加速度レンジ（2, 4, 8, 16[G] を 0, 1, 2, 3 で表す）
        gyro: ジャイロレンジ（250, 500, 1000, 2000[deg/s] を 0, 1, 2, 3 で表す）
    """

    def __init__(self) -> None:
        self.acc: int = 0
        self.gyro: int = 0


class GaitData:
    """
    歩行解析の値（STEP_ANALYSIS のサブヘッダ 0）

    Attributes:
        step_count: 歩数
        gait_type: 歩容タイプ（0:無し、1:歩行、2:走行、3:直立静止）
        direction: ストライド方向（0:なし、1:前方、2:後方、3:内側、4:外側）
        calorie: 総消費カロリー（1要素のタプル）
        distance: 総移動距離（1要素のタプル）
        standing_phase_duration: 立脚期継続時間（1要素のタプル）
        swing_phase_duration: 遊脚期継続時間（1要素のタプル）
    """

    def __init__(self, data: bytes | bytearray) -> None:
        # 2,3は Uint16 で歩数
        self.step_count = int.from_bytes(data[2:4], byteorder='big', signed=False)
        # 公開版は `data[4] & 0b11000000 >> 6` と書かれており、演算子の優先順位で
        # `data[4] & 0b11`（下位2ビット）になる。互換のため同じ値を返す。
        self.gait_type = data[4] & 0b00000011
        # 4番目の 3,4,5 ビット目がストライド方向
        self.direction = (data[4] & 0b00111000) >> 3
        # data[6],data[7]はfloat16で総消費カロリー
        self.calorie = struct.unpack('>e', data[6:8])
        # 8-11はfloat32で総移動距離
        self.distance = struct.unpack('>f', data[8:12])
        # 12-15はfloat32で立脚期継続時間
        self.standing_phase_duration = struct.unpack('>f', data[12:16])
        # 16-19はfloat32で遊脚期継続時間
        self.swing_phase_duration = struct.unpack('>f', data[16:20])

    def print(self) -> None:
        print(f"Step count: {self.step_count}")
        print(f"Gait type: {self.gait_type}")
        print(f"Direction: {self.direction}")
        print(f"Calorie: {self.calorie}")
        print(f"Distance: {self.distance}")
        print(f"Standing phase duration: {self.standing_phase_duration}")
        print(f"Swing phase duration: {self.swing_phase_duration}")


class StrideData:
    """
    ストライドの値（STEP_ANALYSIS のサブヘッダ 1）

    Attributes:
        step_count: 歩数
        foot_angle: フットアングル（1要素のタプル）
        x, y, z: ストライド（1要素のタプル）
    """

    def __init__(self, data: bytes | bytearray) -> None:
        self.step_count = int.from_bytes(data[2:4], byteorder='big', signed=False)
        self.foot_angle = struct.unpack('>f', data[4:8])
        self.x = struct.unpack('>f', data[8:12])
        self.y = struct.unpack('>f', data[12:16])
        self.z = struct.unpack('>f', data[16:20])

    def print(self) -> None:
        print(f"Step count: {self.step_count}")
        print(f"Foot angle: {self.foot_angle}")
        print(f"Stride X: {self.x}")
        print(f"Stride Y: {self.y}")
        print(f"Stride Z: {self.z}")


class PronationData:
    """
    プロネーションの値（STEP_ANALYSIS のサブヘッダ 2）

    Attributes:
        step_count: 歩数
        landing_impact: 着地衝撃力[kgf]（1要素のタプル）
        x, y, z: プロネーション[deg]（1要素のタプル）
    """

    def __init__(self, data: bytes | bytearray) -> None:
        self.step_count = int.from_bytes(data[2:4], byteorder='big', signed=False)
        self.landing_impact = struct.unpack('>f', data[4:8])
        self.x = struct.unpack('>f', data[8:12])
        self.y = struct.unpack('>f', data[12:16])
        self.z = struct.unpack('>f', data[16:20])

    def print(self) -> None:
        print(f"Step count: {self.step_count}")
        print(f"Landing impact: {self.landing_impact}")
        print(f"Pronation X: {self.x}")
        print(f"Pronation Y: {self.y}")
        print(f"Pronation Z: {self.z}")


class QuatDistanceData:
    """
    クォータニオンと移動距離の差分（STEP_ANALYSIS のサブヘッダ 4）

    Attributes:
        step_count: 歩数
        phase: 歩容フェイズ（0:なし、1:立脚期、2:遊脚期）
        period: 歩容ピリオド（0:なし、1:LoadingResponse、2:MidStance、3:TerminalStance、
            4:InitialSwing、5:MidSwing、6:TerminalSwing）
        event: 歩容イベント（0:なし、1:InitialContact、2:FootFlat、3:HeelRise、4:ToeOff、
            5:FeetAdjacent、6:TibiaVertical）
        w, x, y, z: クォータニオン（1要素のタプル）
        x_distance, y_distance, z_distance: 加速度から算出した単位時間の移動距離（1要素のタプル）
    """

    def __init__(self, data: bytes | bytearray) -> None:
        self.step_count = int.from_bytes(data[2:4], byteorder='big', signed=False)
        self.phase = data[4] & 0b00000001
        self.period = (data[4] & 0b00011110) >> 1
        self.event = (data[4] & 0b11100000) >> 5
        # 6-13 は float16 のクォータニオン w, x, y, z
        self.w = struct.unpack('>e', data[6:8])
        self.x = struct.unpack('>e', data[8:10])
        self.y = struct.unpack('>e', data[10:12])
        self.z = struct.unpack('>e', data[12:14])
        # 14-19 は float16 の移動距離 x, y, z
        self.x_distance = struct.unpack('>e', data[14:16])
        self.y_distance = struct.unpack('>e', data[16:18])
        self.z_distance = struct.unpack('>e', data[18:20])

    def print(self) -> None:
        print(f"Step count: {self.step_count}")
        print(f"Phase: {self.phase}")
        print(f"Period: {self.period}")
        print(f"Event: {self.event}")
        print(f"Quat: {self.w}, {self.x}, {self.y}, {self.z}")
        print(
            f"Distance: {self.x_distance}, {self.y_distance}, {self.z_distance}")


class StepCount:
    """STEP_ANALYSIS の種類ごとに最後に受け取った歩数"""

    def __init__(self) -> None:
        self.gait = 0
        self.stride = 0
        self.pronation = 0
        self.quat = 0
