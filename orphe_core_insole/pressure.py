"""
ORPHE INSOLE の個体別圧力校正係数

対応 FW は製造時に測った 6ch ぶんの校正係数を持っている。DEVICE_INFORMATION に [0x10, 0x00, ch] を書くと、
SENSOR_VALUES の notify（ヘッダ 0x39）で 1ch ずつ返る。FW は圧力を含む streaming mode（3 / 4）のときだけ応答する。
ここは応答の解釈と ADC 値 → 圧力[N] の換算だけを持つ。取得の手順は OrpheCoreInsole.read_pressure_calibrations()。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

PRESSURE_SENSOR_COUNT = 6
PRESSURE_CALIBRATION_NOTIFY_HEADER = 0x39
# ヘッダ + ch + func + double × 5
PRESSURE_CALIBRATION_PAYLOAD_LENGTH = 1 + 1 + 1 + 8 * 5
COEFFICIENT_COUNT = 5

REQUEST_COMMAND = 0x10
REQUEST_ACTION = 0x00


@dataclass(frozen=True)
class PressureCalibration:
    """
    1ch ぶんの校正係数

    Attributes:
        func: 換算式の種類。0: c1·exp(c2·x) + c3、1: c1·x⁴ + c2·x³ + c3·x² + c4·x + c5
        coefficients: 係数 (c1, c2, c3, c4, c5)
    """

    func: int
    coefficients: tuple[float, ...]

    @property
    def is_placeholder(self) -> bool:
        """FW に校正値が書かれていないときに返る値（func 0・係数がすべて 1.0）"""
        return (self.func == 0 and len(self.coefficients) == COEFFICIENT_COUNT
                and all(c == 1 for c in self.coefficients))


def encode_pressure_calibration_request(sensor_index: int) -> bytes:
    """1ch ぶんの校正係数を要求するコマンド（DEVICE_INFORMATION に書く）"""
    if not 0 <= sensor_index < PRESSURE_SENSOR_COUNT:
        raise ValueError(f"sensor_index must be 0..{PRESSURE_SENSOR_COUNT - 1}: {sensor_index}")
    return bytes([REQUEST_COMMAND, REQUEST_ACTION, sensor_index])


def is_pressure_calibration_packet(data: bytes | bytearray) -> bool:
    return len(data) > 0 and data[0] == PRESSURE_CALIBRATION_NOTIFY_HEADER


def decode_pressure_calibration(data: bytes | bytearray) -> tuple[int, PressureCalibration] | None:
    """校正係数の notify → (ch, 係数)。ヘッダ違い・長さ不足は None"""
    if not is_pressure_calibration_packet(data) or len(data) < PRESSURE_CALIBRATION_PAYLOAD_LENGTH:
        return None
    coefficients = struct.unpack('>5d', bytes(data[3:3 + 8 * COEFFICIENT_COUNT]))
    return data[1], PressureCalibration(data[2], coefficients)


def apply_pressure_calibration(calibration: PressureCalibration, adc_raw: float) -> float:
    """校正係数で ADC 値を圧力[N] にする。負・NaN・無限大・未知の func は 0"""
    c = calibration.coefficients
    if calibration.func == 0:
        if len(c) < 3:
            return 0.0
        try:
            y = c[0] * math.exp(c[1] * adc_raw) + c[2]
        except OverflowError:
            return 0.0
    elif calibration.func == 1:
        if len(c) < 5:
            return 0.0
        x2 = adc_raw * adc_raw
        x3 = x2 * adc_raw
        x4 = x3 * adc_raw
        y = c[0] * x4 + c[1] * x3 + c[2] * x2 + c[3] * adc_raw + c[4]
    else:
        return 0.0
    if not math.isfinite(y) or y < 0:
        return 0.0
    return y


# 個体別係数を持たない FW 向けの固定式。ADC 値を mV にしてから指数式に通す
LEGACY_C1 = 2.77942
LEGACY_C2 = 2.08348e-3
LEGACY_C3 = 4.14411
LEGACY_THRESHOLD_MV = 240  # これ以下は踏んでいないとして 0
ADC_REFERENCE_VOLT = 3.3
ADC_FULL_SCALE = 4096


def legacy_pressure_to_newton(adc_raw: float) -> float:
    """個体別係数が無いときの換算: mV = adc × 3.3 / 4096 × 1000、240mV 以下は 0、それ以上は c1·exp(c2·mV) + c3"""
    mv = (adc_raw * ADC_REFERENCE_VOLT / ADC_FULL_SCALE) * 1000
    if mv <= LEGACY_THRESHOLD_MV:
        return 0.0
    y = LEGACY_C1 * math.exp(LEGACY_C2 * mv) + LEGACY_C3
    return 0.0 if y < 0 else y


def pressure_to_newton(adc_raw: float, calibration: PressureCalibration | None) -> float:
    """ADC 値を圧力[N] にする。係数が無い・未書き込みなら固定式"""
    if calibration is not None and not calibration.is_placeholder:
        return apply_pressure_calibration(calibration, adc_raw)
    return legacy_pressure_to_newton(adc_raw)
