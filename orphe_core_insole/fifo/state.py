"""
FIFO 収集ループの状態

    NotifyQueue   notify を待てるキュー（応答待ちに使う）
    FifoLoopState 1 収録ぶんの回収済みデータ・再要求キュー・欠損の計上
    DrainBudget   stop() 後の回収フェーズの時間予算

BLE には触らない。recorder.py が組み合わせて使う。
"""

from __future__ import annotations

import asyncio
import time
from collections import deque
from dataclasses import dataclass
from typing import Literal

from .protocol import (
    FIFO_CATCHUP_MAX_BUDGET_FACTOR,
    FIFO_MAX_CARRY_OVER_SERIALS,
    FIFO_RING_BUFFER_CAPACITY,
    UINT16_MAX,
    FifoRequestRange,
    build_requests_from_serials,
    serial_distance,
)


class NotifyQueue:
    """notify を積んでおき、タイムアウト付きで取り出せるキュー"""

    def __init__(self) -> None:
        self._items: deque[bytes] = deque()
        self._event = asyncio.Event()

    def push(self, data: bytes) -> None:
        self._items.append(data)
        self._event.set()

    def drain(self) -> None:
        """溜まっている受信を捨てる（コマンドを送る直前やモード切替直後に使う）"""
        self._items.clear()
        self._event.clear()

    async def wait(self, timeout_sec: float) -> bytes | None:
        """先頭を取り出す。timeout_sec 待っても来なければ None"""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while not self._items:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._event.clear()
            try:
                await asyncio.wait_for(self._event.wait(), remaining)
            except asyncio.TimeoutError:
                return None
        return self._items.popleft()


# 回復できない欠損の原因
#   ring_overflow       追従が間に合わず FW のリングバッファが上書きされた
#   carryover_overflow  再要求キューが上限を超えたので諦めた
#   fw_nodata           再要求したが FW にもうデータが無かった
#   resync_backlog      再同期したとき、要求範囲より古くて取りに行かなかったバックログ
#   stopped_pending     収録を止めるまでに回収できなかった末尾
FifoLossReason = Literal['ring_overflow', 'carryover_overflow', 'fw_nodata', 'resync_backlog', 'stopped_pending']


@dataclass(frozen=True)
class FifoLossEvent:
    reason: FifoLossReason
    dropped: int


class FifoLoopState:
    """ポーリングループ 1 収録ぶんの状態"""

    def __init__(self) -> None:
        # 最後まで要求し終えたシリアル番号。None は未同期（次のポーリングで取り直す）
        self.last_serial: int | None = None
        # 次のループで再要求する範囲（BLE で落ちたぶん）
        self.carry_over: list[FifoRequestRange] = []
        # 回収したパケット（シリアル番号 → 生データ）
        self.raw_store: dict[int, bytes] = {}
        # 回復できずに失った累計シリアル数
        self.dropped = 0
        # まだ通知していない欠損
        self.loss_events: list[FifoLossEvent] = []
        # last_serial が None なのが再同期によるものか（開始直後と区別する）
        self.resync_pending = False
        # 収録範囲の起点と、起点からの最大距離
        self.first_stored_serial: int | None = None
        self.stored_span_max = 0

    def carry_over_count(self) -> int:
        return sum(count for _, count in self.carry_over)

    def note_stored(self, serial: int) -> None:
        """
        格納したシリアル番号で収録範囲を広げる

        起点より手前のシリアル番号が来たら起点を巻き戻す（そうしないと距離が約 65536 になり、
        finalize_pending_loss が存在しない大量の欠損を計上する）。収録範囲は半周未満の前提。
        """
        if self.first_stored_serial is None:
            self.first_stored_serial = serial
            self.stored_span_max = 0
            return
        forward = serial_distance(self.first_stored_serial, serial)
        backward = serial_distance(serial, self.first_stored_serial)
        if backward < forward:
            self.first_stored_serial = serial
            self.stored_span_max += backward
        elif forward > self.stored_span_max:
            self.stored_span_max = forward

    def note_span_target(self, serial: int) -> None:
        """停止時の回収目標まで収録範囲を延ばす（回収できなかった末尾も欠損として数えるため）"""
        if self.first_stored_serial is None:
            return
        forward = serial_distance(self.first_stored_serial, serial)
        if forward > UINT16_MAX / 2:
            return
        self.stored_span_max = max(self.stored_span_max, forward)

    def finalize_pending_loss(self) -> int:
        """
        収録終了時、範囲内で格納も欠損計上もされていないシリアルを stopped_pending として計上する

        これで「範囲内のシリアル数 = 回収数 + dropped」が成り立つ。戻り値は今回計上した数。
        """
        if self.first_stored_serial is None:
            return 0
        pending = self.stored_span_max + 1 - len(self.raw_store) - self.dropped
        if pending <= 0:
            return 0
        self.dropped += pending
        self.loss_events.append(FifoLossEvent('stopped_pending', pending))
        return pending

    def calc_request_range(self, current_serial: int, accumulated: int, max_new_request: int) -> tuple[int, int]:
        """新しく要求する (開始シリアル, 件数)"""
        if self.last_serial is None:
            request_size = min(accumulated, max_new_request)
            # 再同期のときは直近 request_size 件だけ要求するので、それより古いバックログは失われる。
            # 開始直後（resync_pending=False）はバッファを消去済みなので欠損ではない
            if self.resync_pending:
                lost = max(0, accumulated - request_size)
                if lost > 0:
                    self.dropped += lost
                    self.loss_events.append(FifoLossEvent('resync_backlog', lost))
                self.resync_pending = False
            start = (current_serial - (request_size - 1)) % UINT16_MAX if request_size > 0 else 0
            return start, request_size

        need = serial_distance(self.last_serial, current_serial)
        if need > FIFO_RING_BUFFER_CAPACITY:
            # 遅れすぎてリングバッファが上書きされたぶんは回復できない
            skip = need - FIFO_RING_BUFFER_CAPACITY
            self.last_serial = (self.last_serial + skip) % UINT16_MAX
            self.carry_over = []
            self.dropped += skip
            self.loss_events.append(FifoLossEvent('ring_overflow', skip))
            need = serial_distance(self.last_serial, current_serial)
        return (self.last_serial + 1) % UINT16_MAX, min(need, max_new_request)

    def update_after_response(self, ble_loss: set[int], new_no_data: set[int],
                              start_serial: int, request_size: int) -> Literal['resync', 'ok']:
        """応答を受けて last_serial と carry_over を進める"""
        if ble_loss:
            self.carry_over += build_requests_from_serials(ble_loss)
            pending = self.carry_over_count()
            if pending > FIFO_MAX_CARRY_OVER_SERIALS:
                # 再要求しきれないので諦める（回復不能）
                self.dropped += pending
                self.loss_events.append(FifoLossEvent('carryover_overflow', pending))
                self.carry_over = []
                self.last_serial = None
                self.resync_pending = True
                return 'resync'
        if new_no_data:
            # 新しく要求した範囲に no-data → 現在のシリアル番号から取り直す。
            # carry_over は残す（捨てると散発的な欠損がそのまま失われる）
            self.last_serial = None
            self.resync_pending = True
        elif request_size > 0:
            self.last_serial = (start_serial + request_size - 1) % UINT16_MAX
        return 'ok'


class DrainBudget:
    """
    「無音が budget_sec 続いたら諦める」時間予算

    データが届く間は note_progress() で期限が延び、budget_sec × 10 の絶対期限で打ち切る。
    """

    def __init__(self, budget_sec: float, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        self.budget_sec = max(0.0, budget_sec)
        self.deadline = now + self.budget_sec
        self.hard_deadline = now + self.budget_sec * FIFO_CATCHUP_MAX_BUDGET_FACTOR

    @property
    def expired(self) -> bool:
        return self.remaining_sec() <= 0

    def remaining_sec(self) -> float:
        return max(0.0, min(self.deadline, self.hard_deadline) - time.monotonic())

    def note_progress(self) -> None:
        self.deadline = min(time.monotonic() + self.budget_sec, self.hard_deadline)
