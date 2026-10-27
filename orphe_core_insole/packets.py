"""
通知パケットのパース結果と、CORE / INSOLE 共通の STEP_ANALYSIS パーサー

SENSOR_VALUES のレイアウトはデバイスごとに違うので profiles/ 側でパースする。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .data import (
    AccData,
    GaitData,
    GyroData,
    PressureData,
    PronationData,
    QuatData,
    QuatDistanceData,
    StepCount,
    StrideData,
)


@dataclass
class SensorFrame:
    """SENSOR_VALUES パケットに含まれる 1 サンプル"""

    acc: AccData
    converted_acc: AccData
    gyro: GyroData
    converted_gyro: GyroData
    quat: QuatData | None = None
    pressure: PressureData | None = None


@dataclass
class SensorValuesPacket:
    """
    SENSOR_VALUES パケット 1 つ分

    Attributes:
        header: data[0]（CORE: 40 / 50、INSOLE: 50 / 55 / 56）
        serial_number: パケットのシリアル番号。ヘッダ 40（CORE 50Hz モデル）は None
        timestamp: 先頭のタイムスタンプ
        frames: コールバックに渡す順のサンプル
    """

    header: int
    serial_number: int | None
    timestamp: int
    frames: list[SensorFrame] = field(default_factory=list)


def stamp(targets: list, timestamp: int, serial_number: int, packet_number: int) -> None:
    """データクラスにタイムスタンプ・シリアル番号・パケット番号を入れる"""
    for target in targets:
        target.timestamp = timestamp
        target.serial_number = serial_number
        target.packet_number = packet_number


# STEP_ANALYSIS のサブヘッダ（data[1]）とコールバック名
STEP_ANALYSIS_GAIT = 0
STEP_ANALYSIS_STRIDE = 1
STEP_ANALYSIS_PRONATION = 2
STEP_ANALYSIS_QUAT_DISTANCE = 4


def parse_step_analysis(
    data: bytes | bytearray, step_count: StepCount
) -> tuple[str, GaitData | StrideData | PronationData | QuatDistanceData] | None:
    """
    STEP_ANALYSIS パケットをパースする

    data[1] のサブヘッダで種類が決まる（0: Gait Overview、1: Stride、2: Pronation、4: クォータニオン）。
    0, 1, 2 は到着を担保するため同じデータが 2 回続けて届くので、歩数が増えたときだけ返す。
    step_count はここで更新する。

    Returns:
        (イベント名, データ) のタプル。通知しないときは None
    """
    sub_header = data[1]
    count = int.from_bytes(data[2:4], byteorder='big', signed=False)

    if sub_header == STEP_ANALYSIS_GAIT:
        is_new = count > step_count.gait
        step_count.gait = count
        return ('gait', GaitData(data)) if is_new else None
    if sub_header == STEP_ANALYSIS_STRIDE:
        is_new = count > step_count.stride
        step_count.stride = count
        return ('stride', StrideData(data)) if is_new else None
    if sub_header == STEP_ANALYSIS_PRONATION:
        is_new = count > step_count.pronation
        step_count.pronation = count
        return ('pronation', PronationData(data)) if is_new else None
    if sub_header == STEP_ANALYSIS_QUAT_DISTANCE:
        # クォータニオンは歩数に関係なく毎回届く
        step_count.quat = count
        return ('quat_distance', QuatDistanceData(data))
    return None
