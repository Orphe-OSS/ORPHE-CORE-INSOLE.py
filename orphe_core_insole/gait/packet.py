"""
STEP_ANALYSIS パケットの型とデコード（INSOLE の歩行解析）

パケット（20 バイト、big-endian）: data[0]=51, data[1]=サブヘッダ
（0=概要 / 1=ストライド / 2=プロネーション / 4=motion）, data[2..3]=step_number。
状態を持たない関数だけを置く。
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Literal

# STEP_ANALYSIS パケットのヘッダ
GAIT_PACKET_HEADER = 51
# STEP_ANALYSIS パケットの長さ[byte]
GAIT_PACKET_LENGTH = 20

# 歩行タイプの分類値 → 名前
GAIT_TYPES = ('none', 'walk', 'run', 'stance')
# ストライド方向の分類値 → 名前
STRIDE_DIRECTIONS = ('none', 'forward', 'backward', 'inside', 'outside')

# 接地パターン・プロネーションの分類しきい値（FW 側の解析と揃える）
FOOT_STRIKE_MID_THRESHOLD = -3.0
FOOT_STRIKE_FORE_THRESHOLD = 2.0
PRONATION_AVERAGE = -9.4
PRONATION_STD = 3.5


# ── サニタイズ ───────────────────────────────────────────────────

def _sanitize(value: float) -> float | None:
    """NaN / Inf は「値が未確定」なので None にする"""
    return value if math.isfinite(value) else None


def _sanitize_non_negative(value: float) -> float | None:
    """
    本来は負にならない量（時間・距離・カロリー・衝撃）向け

    FW は未確定の値に -1 を入れることがあるので、負の値も None にする
    （角度・ベクトルなど負の値が正当な項目には使わない）。
    """
    sanitized = _sanitize(value)
    return None if sanitized is None or sanitized < 0 else sanitized


def _f16(data: bytes, at: int) -> float:
    return struct.unpack_from('>e', data, at)[0]


def _f32(data: bytes, at: int) -> float:
    return struct.unpack_from('>f', data, at)[0]


# ── 分類 ─────────────────────────────────────────────────────────

def gait_type_to_str(value: int) -> str:
    """歩行タイプの分類値 → 名前（GAIT_TYPES）。範囲外は 'unknown'"""
    return GAIT_TYPES[value] if 0 <= value < len(GAIT_TYPES) else 'unknown'


def stride_direction_to_str(value: int) -> str:
    """ストライド方向の分類値 → 名前（STRIDE_DIRECTIONS）。範囲外は 'unknown'"""
    return STRIDE_DIRECTIONS[value] if 0 <= value < len(STRIDE_DIRECTIONS) else 'unknown'


def foot_strike_to_str(strike_angle: float | None) -> str:
    """着地時の足の角度（pronation_x）から接地パターンを決める"""
    if strike_angle is None:
        return 'none'
    if strike_angle > FOOT_STRIKE_FORE_THRESHOLD:
        return 'forefoot'
    if strike_angle > FOOT_STRIKE_MID_THRESHOLD:
        return 'midfoot'
    return 'heelStrike'


def pronation_to_str(pronation_y: float | None) -> str:
    """プロネーション角（pronation_y）から種別を決める"""
    if pronation_y is None:
        return 'none'
    ave, std = PRONATION_AVERAGE, PRONATION_STD
    if ave - std <= pronation_y <= ave + std:
        return 'neutral'
    if ave + std < pronation_y <= ave + std * 3:
        return 'over'
    if pronation_y > ave + std * 3:
        return 'severeOver'
    if ave - std * 3 <= pronation_y < ave - std:
        return 'under'
    if pronation_y < ave - std * 3:
        return 'severeUnder'
    return 'none'


# ── パケット ─────────────────────────────────────────────────────

@dataclass(frozen=True)
class GaitOverviewPacket:
    """歩行の概要（サブヘッダ 0）。None は FW 側で未確定の値"""

    step_number: int
    gait_type: str                  # 歩行タイプ（GAIT_TYPES）
    stride_direction: str           # ストライド方向（STRIDE_DIRECTIONS）
    calorie: float | None           # 消費カロリー
    distance_m: float | None        # 距離[m]
    stance_phase_s: float | None    # 立脚期[s]
    swing_phase_s: float | None     # 遊脚期[s]
    type: Literal['overview'] = 'overview'
    subheader: int = 0


@dataclass(frozen=True)
class GaitStridePacket:
    """ストライド（サブヘッダ 1）"""

    step_number: int
    foot_angle: float | None    # 着地時の足の角度[deg]
    stride_x: float | None      # ストライド X[m]
    stride_y: float | None      # ストライド Y[m]
    stride_z: float | None      # ストライド Z[m]
    type: Literal['stride'] = 'stride'
    subheader: int = 1


@dataclass(frozen=True)
class GaitPronationPacket:
    """プロネーション（サブヘッダ 2）"""

    step_number: int
    landing_force: float | None     # 着地の衝撃
    pronation_x: float | None       # 着地角度 X[deg]（接地パターンの判定に使う）
    pronation_y: float | None       # プロネーション角 Y[deg]（種別の判定に使う）
    pronation_z: float | None       # プロネーション角 Z[deg]
    type: Literal['pronation'] = 'pronation'
    subheader: int = 2


@dataclass(frozen=True)
class GaitMotionPacket:
    """
    motion（サブヘッダ 4）

    1 歩ごとではなく、STEP_ANALYSIS の通知のたびに届く連続値（姿勢のクォータニオンと変位の差分）。
    """

    step_number: int
    gait_cycle_phase: int       # 歩行周期のフェーズ
    gait_cycle_period: int      # 歩行周期の区間
    gait_cycle_event: int       # 歩行周期のイベント
    quat_w: float | None
    quat_x: float | None
    quat_y: float | None
    quat_z: float | None
    delta_x: float | None       # 前のパケットからの変位 X[m]
    delta_y: float | None       # 前のパケットからの変位 Y[m]
    delta_z: float | None       # 前のパケットからの変位 Z[m]
    type: Literal['motion'] = 'motion'
    subheader: int = 4


GaitPacket = GaitOverviewPacket | GaitStridePacket | GaitPronationPacket | GaitMotionPacket


def _decode_overview(data: bytes, step: int) -> GaitOverviewPacket:
    flags = data[4]
    return GaitOverviewPacket(
        step_number=step,
        gait_type=gait_type_to_str((flags >> 6) & 0b11),
        stride_direction=stride_direction_to_str((flags >> 3) & 0b111),
        calorie=_sanitize_non_negative(_f16(data, 6)),
        distance_m=_sanitize_non_negative(_f32(data, 8)),
        stance_phase_s=_sanitize_non_negative(_f32(data, 12)),
        swing_phase_s=_sanitize_non_negative(_f32(data, 16)),
    )


def _decode_stride(data: bytes, step: int) -> GaitStridePacket:
    return GaitStridePacket(
        step_number=step,
        foot_angle=_sanitize(_f32(data, 4)),
        stride_x=_sanitize(_f32(data, 8)),
        stride_y=_sanitize(_f32(data, 12)),
        stride_z=_sanitize(_f32(data, 16)),
    )


def _decode_pronation(data: bytes, step: int) -> GaitPronationPacket:
    return GaitPronationPacket(
        step_number=step,
        landing_force=_sanitize_non_negative(_f32(data, 4)),
        pronation_x=_sanitize(_f32(data, 8)),
        pronation_y=_sanitize(_f32(data, 12)),
        pronation_z=_sanitize(_f32(data, 16)),
    )


def _decode_motion(data: bytes, step: int) -> GaitMotionPacket:
    flags = data[4]
    return GaitMotionPacket(
        step_number=step,
        gait_cycle_phase=(flags >> 6) & 0b11,
        gait_cycle_period=(flags >> 3) & 0b111,
        gait_cycle_event=flags & 0b111,
        quat_w=_sanitize(_f16(data, 6)),
        quat_x=_sanitize(_f16(data, 8)),
        quat_y=_sanitize(_f16(data, 10)),
        quat_z=_sanitize(_f16(data, 12)),
        delta_x=_sanitize(_f16(data, 14)),
        delta_y=_sanitize(_f16(data, 16)),
        delta_z=_sanitize(_f16(data, 18)),
    )


_DECODERS = {0: _decode_overview, 1: _decode_stride, 2: _decode_pronation, 4: _decode_motion}


def decode_gait_packet(data: bytes | bytearray) -> GaitPacket | None:
    """
    歩行解析のパケット（20 バイト）を 1 つデコードする

    歩行解析のパケットでなければ（ヘッダ違い・長さ不足・知らないサブヘッダ）None。
    """
    if len(data) < GAIT_PACKET_LENGTH or data[0] != GAIT_PACKET_HEADER:
        return None
    decoder = _DECODERS.get(data[1])
    if decoder is None:
        return None
    return decoder(bytes(data), int.from_bytes(data[2:4], byteorder='big'))
