"""
サブパケットを 1 歩ずつまとめ、1 歩ぶんの歩行パラメーター（GaitRow）を作る

overview / stride / pronation は取りこぼし対策で 2 回ずつ届くので、step_number ごとにまとめて
重複を除き、3 種類そろった歩だけを出す。歩の欠け（incomplete / gap / jump）の統計もここで持つ。
"""

from __future__ import annotations

import contextlib
import math
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Literal

from ..fifo.protocol import _to_fixed
from .packet import (
    GaitOverviewPacket,
    GaitPronationPacket,
    GaitStridePacket,
    foot_strike_to_str,
    pronation_to_str,
)

# そろわないまま溜める歩の上限（メモリ保護）
MAX_PENDING_STEPS = 64
# FW は overview / stride / pronation を 2 回ずつ、その歩の直後にまとめて送る。
# この歩数ぶん先の step が届いた時点でそろっていない歩は、もうそろわない（incomplete）とみなす
GAIT_STALE_STEP_DISTANCE = 8
# これを超える step の前進は、欠け（gap）ではなく番号の飛び（jump）として扱う
GAIT_GAP_COUNT_LIMIT = 64
# gap として数えた歩を覚えておく数（後から届いたら数え直すため）
MISSED_STEP_MEMORY = 256
UINT16_MAX = 0x10000

GAIT_CSV_HEADER = (
    'step_number,gait_type,stride_direction,distance_m,'
    'stance_phase_s,swing_phase_s,duration_s,cadence_hz,speed_mps,'
    'foot_angle_deg,stride_x_m,stride_y_m,stride_z_m,stride_norm_m,'
    'landing_force,strike_angle_deg,foot_strike,'
    'pronation_deg,pronation_type,pronation_z_deg,calorie'
)


def step_distance(start: int, end: int) -> int:
    """start から end までの前進量（65535 → 0 の巻き戻りをまたいでも正しく数える）"""
    return (end - start) % UINT16_MAX


@dataclass(frozen=True)
class GaitRow:
    """1 歩ぶんの歩行パラメーター（overview + stride + pronation と、そこから計算した値）"""

    step_number: int
    gait_type: str                      # 歩行タイプ（GAIT_TYPES）
    stride_direction: str               # ストライド方向（STRIDE_DIRECTIONS）
    distance_m: float | None            # 距離[m]
    stance_phase_s: float | None        # 立脚期[s]
    swing_phase_s: float | None         # 遊脚期[s]
    duration_s: float | None            # 1 歩の時間[s]（立脚期 + 遊脚期）
    cadence_hz: float | None            # ケイデンス[Hz]（1 / duration_s）
    speed_mps: float | None             # 速さ[m/s]（stride_norm_m / duration_s）
    foot_angle_deg: float | None        # 着地時の足の角度[deg]
    stride_x_m: float | None
    stride_y_m: float | None
    stride_z_m: float | None
    stride_norm_m: float | None         # ストライドの長さ[m]
    landing_force: float | None         # 着地の衝撃
    strike_angle_deg: float | None      # 着地角度 X[deg]（接地パターンの元の値）
    foot_strike: str                    # 接地パターン（none / heelStrike / midfoot / forefoot）
    pronation_deg: float | None         # プロネーション角 Y[deg]（種別の元の値）
    pronation_type: str                 # プロネーション種別（none / neutral / over / severeOver / under / severeUnder）
    pronation_z_deg: float | None       # プロネーション角 Z[deg]
    calorie: float | None               # 消費カロリー


@dataclass(frozen=True)
class GaitIncompleteStep:
    """サブパケットがそろわないまま古くなった歩"""

    step_number: int
    missing: tuple[str, ...]    # 届かなかったサブパケット（overview / stride / pronation）
    reason: Literal['incomplete'] = 'incomplete'


@dataclass(frozen=True)
class GaitStepGap:
    """step_number が飛んで、1 つも届かなかった歩"""

    steps: tuple[int, ...]
    count: int
    reason: Literal['gap'] = 'gap'


@dataclass(frozen=True)
class GaitStepNumberJump:
    """前進が大きすぎるので、欠けではなく番号の飛びとみなした"""

    from_step: int
    to_step: int
    forward: int
    reason: Literal['step_number_jump'] = 'step_number_jump'


GaitStepLossInfo = GaitIncompleteStep | GaitStepGap | GaitStepNumberJump


@dataclass
class GaitMissingParts:
    """incomplete の内訳（どのサブパケットが何回足りなかったか）"""

    overview: int = 0
    stride: int = 0
    pronation: int = 0


@dataclass
class GaitStepLossStats:
    """歩の欠けの統計"""

    completed_steps: int = 0        # 3 種類そろって出せた歩数
    incomplete_steps: int = 0       # そろわないまま諦めた歩数
    missing_parts: GaitMissingParts = field(default_factory=GaitMissingParts)
    gap_steps: int = 0              # 1 つも届かなかった歩数
    jumps: int = 0                  # 番号の飛びとみなした回数
    last_seen_step: int | None = None
    pending_steps: int = 0          # そろうのを待っている歩数


def stride_norm(stride: GaitStridePacket) -> float | None:
    """ストライドの長さ（1 歩の移動量）。成分が欠けていれば None"""
    x, y, z = stride.stride_x, stride.stride_y, stride.stride_z
    if x is None or y is None or z is None:
        return None
    return math.sqrt(x * x + y * y + z * z)


def build_gait_row(step_number: int, overview: GaitOverviewPacket, stride: GaitStridePacket,
                   pronation: GaitPronationPacket) -> GaitRow:
    """overview / stride / pronation を 1 歩ぶんにまとめ、時間・ケイデンス・速さも計算する"""
    stance = overview.stance_phase_s
    swing = overview.swing_phase_s
    duration = stance + swing if stance is not None and swing is not None else None
    if duration is not None and duration <= 0:
        duration = None
    norm = stride_norm(stride)
    return GaitRow(
        step_number=step_number,
        gait_type=overview.gait_type,
        stride_direction=overview.stride_direction,
        distance_m=overview.distance_m,
        stance_phase_s=stance,
        swing_phase_s=swing,
        duration_s=duration,
        cadence_hz=1.0 / duration if duration is not None else None,
        speed_mps=norm / duration if duration is not None and norm is not None else None,
        foot_angle_deg=stride.foot_angle,
        stride_x_m=stride.stride_x,
        stride_y_m=stride.stride_y,
        stride_z_m=stride.stride_z,
        stride_norm_m=norm,
        landing_force=pronation.landing_force,
        strike_angle_deg=pronation.pronation_x,
        foot_strike=foot_strike_to_str(pronation.pronation_x),
        pronation_deg=pronation.pronation_y,
        pronation_type=pronation_to_str(pronation.pronation_y),
        pronation_z_deg=pronation.pronation_z,
        calorie=overview.calorie,
    )


def _csv_cell(value: float | str | None) -> str:
    if value is None:
        return ''
    if isinstance(value, str):
        return value
    if float(value).is_integer():
        return str(int(value))
    return _to_fixed(value, 4)


def gait_row_to_csv(row: GaitRow) -> str:
    """GaitRow を GAIT_CSV_HEADER の並びで CSV の 1 行にする（整数はそのまま、小数は 4 桁、None は空欄）"""
    return ','.join(_csv_cell(value) for value in (
        row.step_number, row.gait_type, row.stride_direction, row.distance_m,
        row.stance_phase_s, row.swing_phase_s, row.duration_s, row.cadence_hz, row.speed_mps,
        row.foot_angle_deg, row.stride_x_m, row.stride_y_m, row.stride_z_m, row.stride_norm_m,
        row.landing_force, row.strike_angle_deg, row.foot_strike,
        row.pronation_deg, row.pronation_type, row.pronation_z_deg, row.calorie,
    ))


@dataclass
class _PendingParts:
    overview: GaitOverviewPacket | None = None
    stride: GaitStridePacket | None = None
    pronation: GaitPronationPacket | None = None


class GaitAggregator:
    """
    サブパケットを step_number ごとにまとめ、overview / stride / pronation がそろった歩を返す

    サブパケットは複数回届くので、同じ歩は 1 回だけ出す。歩の欠け（incomplete / gap / jump）も数えて知らせる。
    """

    def __init__(self) -> None:
        # dict は挿入順（= 届いた順）を保つ。step_number は巻き戻るので、古さは値ではなく順番で決める
        self._pending: dict[int, _PendingParts] = {}
        self._emitted: dict[int, None] = {}
        self._missed_steps: dict[int, None] = {}
        self._last_seen_step: int | None = None
        self._stats = GaitStepLossStats()
        # 歩の欠けが決まったときに呼ぶ関数（InsoleGait がつなぐ）
        self.on_step_loss: Callable[[GaitStepLossInfo], None] | None = None

    def stats(self) -> GaitStepLossStats:
        """欠けの統計（コピー）"""
        stats = self._stats
        return GaitStepLossStats(
            completed_steps=stats.completed_steps,
            incomplete_steps=stats.incomplete_steps,
            missing_parts=GaitMissingParts(**vars(stats.missing_parts)),
            gap_steps=stats.gap_steps,
            jumps=stats.jumps,
            last_seen_step=self._last_seen_step,
            pending_steps=len(self._pending),
        )

    def add(self, packet: GaitOverviewPacket | GaitStridePacket | GaitPronationPacket) -> GaitRow | None:
        """サブパケットを 1 つ取り込む。その歩の 3 種類がそろったら GaitRow を返す"""
        step = packet.step_number
        if step in self._emitted:
            return None  # 2 回目の送信

        # gap として数えた歩が遅れて届いたら、数え直す
        if step in self._missed_steps:
            del self._missed_steps[step]
            self._stats.gap_steps = max(0, self._stats.gap_steps - 1)

        is_new_step = step not in self._pending
        parts = self._pending.setdefault(step, _PendingParts())
        if isinstance(packet, GaitOverviewPacket):
            parts.overview = packet
        elif isinstance(packet, GaitStridePacket):
            parts.stride = packet
        else:
            parts.pronation = packet

        if is_new_step:
            self._note_step_seen(step)

        row = None
        if parts.overview is not None and parts.stride is not None and parts.pronation is not None:
            row = build_gait_row(step, parts.overview, parts.stride, parts.pronation)
            del self._pending[step]
            self._mark_emitted(step)
            self._stats.completed_steps += 1
        self._evict_stale_pending()
        self._evict_old_pending()
        return row

    def _note_step_seen(self, step: int) -> None:
        """
        step_number の前進から、1 つも届かなかった歩（gap）を見つける

        前進が GAIT_GAP_COUNT_LIMIT を超えるときは番号の飛びとして扱う（FW の再起動などで番号は戻る）。
        """
        last = self._last_seen_step
        if last is None:
            self._last_seen_step = step
            return
        forward = step_distance(last, step)
        if forward == 0:
            return
        if forward > GAIT_GAP_COUNT_LIMIT:
            if step_distance(step, last) <= GAIT_GAP_COUNT_LIMIT:
                return  # 少し前の歩が遅れて届いた
            self._stats.jumps += 1
            self._last_seen_step = step
            self._notify_loss(GaitStepNumberJump(from_step=last, to_step=step, forward=forward))
            return
        if forward > 1:
            missed = []
            for i in range(1, forward):
                missed_step = (last + i) % UINT16_MAX
                if missed_step in self._pending or missed_step in self._emitted:
                    continue
                missed.append(missed_step)
                self._missed_steps[missed_step] = None
                while len(self._missed_steps) > MISSED_STEP_MEMORY:
                    del self._missed_steps[next(iter(self._missed_steps))]
            if missed:
                self._stats.gap_steps += len(missed)
                self._notify_loss(GaitStepGap(steps=tuple(missed), count=len(missed)))
        self._last_seen_step = step

    def _evict_stale_pending(self) -> None:
        """最新の step より GAIT_STALE_STEP_DISTANCE 以上古い歩は再送も終わっているので、incomplete として捨てる"""
        if self._last_seen_step is None:
            return
        for step, parts in list(self._pending.items()):
            if step_distance(step, self._last_seen_step) >= GAIT_STALE_STEP_DISTANCE:
                del self._pending[step]
                self._count_incomplete(step, parts)

    def _evict_old_pending(self) -> None:
        while len(self._pending) > MAX_PENDING_STEPS:
            oldest = next(iter(self._pending))
            self._count_incomplete(oldest, self._pending.pop(oldest))

    def _count_incomplete(self, step: int, parts: _PendingParts) -> None:
        self._stats.incomplete_steps += 1
        missing = []
        for name in ('overview', 'stride', 'pronation'):
            if getattr(parts, name) is None:
                setattr(self._stats.missing_parts, name, getattr(self._stats.missing_parts, name) + 1)
                missing.append(name)
        self._notify_loss(GaitIncompleteStep(step_number=step, missing=tuple(missing)))

    def _mark_emitted(self, step: int) -> None:
        self._emitted[step] = None
        while len(self._emitted) > MAX_PENDING_STEPS * 4:
            del self._emitted[next(iter(self._emitted))]

    def _notify_loss(self, info: GaitStepLossInfo) -> None:
        if self.on_step_loss is None:
            return
        # 知らせる側の例外で集計を止めない
        with contextlib.suppress(Exception):
            self.on_step_loss(info)
