"""
InsoleGait — INSOLE の歩行解析をリアルタイムに受け取る

INSOLE の FW は歩いている間の歩行パラメーター（ストライド・立脚期 / 遊脚期・接地パターン・
プロネーションなど）を計算し、STEP_ANALYSIS で通知する。購読するだけでよい（モードの変更は要らない）。

    gait = InsoleGait(device)                  # connect() 済みの OrpheCoreInsole（INSOLE）
    gait.on('gait', lambda row: print(row))
    if await gait.start():
        await asyncio.sleep(60)
        await gait.stop()
        gait.save_csv('gait.csv')

歩行解析中は STEP_ANALYSIS の notify を横取りするので、OrpheCoreInsole の gait / stride /
pronation / quat_distance イベントは出ない。1 つの OrpheCoreInsole で同時に動かせる InsoleGait は 1 つ。
"""

from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from ..uuids import CHARACTERISTIC_STEP_ANALYSIS_UUID
from .aggregator import GAIT_CSV_HEADER, GaitAggregator, GaitRow, GaitStepLossInfo, GaitStepLossStats, gait_row_to_csv
from .packet import GaitMotionPacket, GaitPacket, decode_gait_packet

if TYPE_CHECKING:
    from ..device import OrpheCoreInsole

logger = logging.getLogger(__name__)

# on() で購読できるイベント
GAIT_EVENTS = ('gait', 'motion', 'raw', 'transport', 'diagnostic', 'step_loss', 'error')

# wait_for_packet() の既定の待ち時間[s]
DEFAULT_PACKET_TIMEOUT_SEC = 1.2


@dataclass(frozen=True)
class GaitTransportInfo:
    """notify を 1 つ受け取るたびの記録（購読できているかを確かめる用）"""

    received_at: int                # 受け取った時刻（epoch ms）
    byte_length: int
    header: int | None              # 先頭バイト（歩行解析のパケットなら 51）
    subheader: int | None
    valid: bool                     # 歩行解析のパケットとしてデコードできたか
    transport_notifications: int    # ここまでの notify の数
    valid_packets: int              # ここまでの有効なパケットの数
    invalid_packets: int            # ここまでの無効なパケットの数


@dataclass(frozen=True)
class GaitDiagnostics:
    """InsoleGait の状態"""

    running: bool                   # start() 済みか
    subscribed: bool                # STEP_ANALYSIS を購読できているか
    transport_notifications: int
    valid_packets: int
    invalid_packets: int
    last_transport_at: int | None   # 最後に notify を受けた時刻（epoch ms）
    last_valid_packet_at: int | None
    last_transport: GaitTransportInfo | None
    step_loss: GaitStepLossStats


class InsoleGait:
    """
    INSOLE の歩行解析

    Args:
        device: connect() 済みの OrpheCoreInsole（INSOLE）
    """

    def __init__(self, device: OrpheCoreInsole) -> None:
        self.device = device
        self.aggregator = GaitAggregator()
        # そろった歩（CSV に出す）
        self.rows: list[GaitRow] = []
        self._listeners: dict[str, list[Callable[..., Any]]] = {event: [] for event in GAIT_EVENTS}
        self._lock = asyncio.Lock()
        self._running = False
        self._subscribed = False
        self._remove_sink: Callable[[], None] | None = None
        self._remove_disconnect: Callable[[], None] | None = None
        self._transport_notifications = 0
        self._valid_packets = 0
        self._invalid_packets = 0
        self._last_transport_at: int | None = None
        self._last_valid_packet_at: int | None = None
        self._last_transport: GaitTransportInfo | None = None
        self._packet_waiters: list[tuple[int, asyncio.Future[bool]]] = []
        self._reported_invalid: set[tuple[int, int | None, int | None]] = set()
        self._attach_aggregator()

    # ── イベント ──────────────────────────────────────────────

    def on(self, event: str, callback: Callable[..., Any]) -> Callable[[], None]:
        """
        イベントを購読する。戻り値を呼ぶと購読をやめる

        gait(GaitRow)                   1 歩ぶんがそろったとき
        motion(GaitMotionPacket)        姿勢と変位の連続値が届くたび
        raw(GaitPacket)                 デコードできたパケットすべて
        transport(GaitTransportInfo)    notify を受け取るたび
        diagnostic(dict)                購読の開始・切断・待ち時間切れなど（'type' に種類）
        step_loss(GaitStepLossInfo)     歩の欠けが決まったとき
        error(Exception)                エラー
        """
        if event not in self._listeners:
            raise ValueError(f"unknown event: {event!r} (choose from {', '.join(GAIT_EVENTS)})")
        self._listeners[event].append(callback)
        return lambda: self._listeners[event].remove(callback) if callback in self._listeners[event] else None

    def _emit(self, event: str, *args: Any) -> None:
        for callback in list(self._listeners[event]):
            try:
                callback(*args)
            except Exception as error:
                if event == 'error':
                    logger.exception("gait error listener failed")
                else:
                    self._report_error(error)

    def _report_error(self, error: BaseException) -> None:
        if self._listeners['error']:
            self._emit('error', error)
        else:
            logger.warning("gait: %s", error)

    def _emit_diagnostic(self, type_: str, **detail: Any) -> None:
        if self._listeners['diagnostic']:
            self._emit('diagnostic', {'type': type_, **detail, 'diagnostics': self.diagnostics()})

    # ── 状態 ──────────────────────────────────────────────────

    @property
    def step_count(self) -> int:
        """そろった歩数（rows の件数）"""
        return len(self.rows)

    @property
    def is_running(self) -> bool:
        return self._running

    def diagnostics(self) -> GaitDiagnostics:
        """購読の状態と受信の数（購読できたことと、データが実際に届いていることを分けて確かめる）"""
        return GaitDiagnostics(
            running=self._running,
            subscribed=self._subscribed,
            transport_notifications=self._transport_notifications,
            valid_packets=self._valid_packets,
            invalid_packets=self._invalid_packets,
            last_transport_at=self._last_transport_at,
            last_valid_packet_at=self._last_valid_packet_at,
            last_transport=self._last_transport,
            step_loss=self.aggregator.stats(),
        )

    def reset(self) -> None:
        """集めた歩と受信の数を捨てる。解析中は何もしない（stop() してから呼ぶ）"""
        if self._running:
            return
        self.aggregator = GaitAggregator()
        self._attach_aggregator()
        self.rows = []
        self._transport_notifications = 0
        self._valid_packets = 0
        self._invalid_packets = 0
        self._last_transport_at = None
        self._last_valid_packet_at = None
        self._last_transport = None
        self._reported_invalid.clear()

    async def wait_for_packet(self, after_count: int | None = None,
                              timeout_sec: float = DEFAULT_PACKET_TIMEOUT_SEC) -> bool:
        """
        有効なパケットが届くまで待つ（購読を始めただけで受信できたと思い込まないために使う）

        Args:
            after_count: 有効なパケットの数がこれを超えるまで待つ（既定は今の数）
            timeout_sec: 待つ時間[s]
        Returns:
            時間内に届いたら True
        """
        after = self._valid_packets if after_count is None else after_count
        if self._valid_packets > after:
            return True
        if not self._running or not self._subscribed or timeout_sec <= 0:
            return False
        waiter = (after, asyncio.get_running_loop().create_future())
        self._packet_waiters.append(waiter)
        try:
            return await asyncio.wait_for(waiter[1], timeout_sec)
        except asyncio.TimeoutError:
            self._emit_diagnostic('packet-timeout', after_count=after, timeout_sec=timeout_sec)
            return False
        finally:
            if waiter in self._packet_waiters:
                self._packet_waiters.remove(waiter)

    # ── 開始・停止 ────────────────────────────────────────────

    async def start(self) -> bool:
        """
        歩行解析の通知を受け取り始める（start() / stop() は順に実行される）

        Returns:
            始められたら True
        """
        async with self._lock:
            return await self._start()

    async def stop(self) -> None:
        """歩行解析の通知を止める"""
        async with self._lock:
            await self._stop()

    async def refresh_subscription(self) -> bool:
        """STEP_ANALYSIS をいったん止めて購読し直す（集めた歩はそのまま）"""
        async with self._lock:
            if not self._running:
                return False
            was_subscribed = self._subscribed
            self._subscribed = False
            if was_subscribed and self.device.is_connected():
                try:
                    await self.device.stop_step_analysis_notification()
                except Exception as error:  # noqa: BLE001
                    self._report_error(error)
            if not self._running:
                return False
            return await self._subscribe()

    async def _start(self) -> bool:
        if self._running and self._subscribed:
            return True
        if not self.device.is_connected():
            self._report_error(RuntimeError("InsoleGait.start(): insole is not connected"))
            return False
        if not self._running:
            self.aggregator = GaitAggregator()
            self._attach_aggregator()
            self.rows = []
        if self._remove_sink is None:
            try:
                # 同じ OrpheCoreInsole でほかの InsoleGait が動いていればここで失敗する
                self._remove_sink = self.device.set_notify_sink(self._on_packet, CHARACTERISTIC_STEP_ANALYSIS_UUID)
            except RuntimeError as error:
                self._report_error(error)
                return False
        self._running = True
        if self._remove_disconnect is None:
            self._remove_disconnect = self.device.on('disconnect', self._on_disconnect)

        ok = await self._subscribe()
        if not ok and not self._subscribed:
            self._running = False
            self._remove_hooks()
        return ok

    async def _stop(self) -> None:
        was_subscribed = self._subscribed
        was_running = self._running or was_subscribed
        self._running = False
        self._subscribed = False
        self._remove_hooks()
        self._resolve_packet_waiters(False)
        if not was_running:
            return
        if was_subscribed and self.device.is_connected():
            try:
                await self.device.stop_step_analysis_notification()
            except Exception:  # noqa: BLE001, S110 (止められなくても停止扱いにする)
                pass

    async def _subscribe(self) -> bool:
        try:
            await self.device.start_step_analysis_notification()
        except Exception as error:  # noqa: BLE001
            if self._running:
                self._report_error(error)
            return False
        if not self._running:
            return False
        self._subscribed = True
        self._emit_diagnostic('subscription-started')
        return True

    def _on_disconnect(self) -> None:
        self._subscribed = False
        self._resolve_packet_waiters(False)
        self._emit_diagnostic('physical-disconnect')

    def _remove_hooks(self) -> None:
        if self._remove_sink is not None:
            self._remove_sink()
            self._remove_sink = None
        if self._remove_disconnect is not None:
            self._remove_disconnect()
            self._remove_disconnect = None

    # ── 受信 ──────────────────────────────────────────────────

    def _on_packet(self, data: bytes) -> None:
        received_at = int(time.time() * 1000)
        header = data[0] if len(data) > 0 else None
        subheader = data[1] if len(data) > 1 else None
        self._transport_notifications += 1
        self._last_transport_at = received_at
        packet: GaitPacket | None = None
        try:
            packet = decode_gait_packet(data)
        except Exception as error:  # noqa: BLE001
            self._report_error(error)
        valid = packet is not None
        info = GaitTransportInfo(
            received_at=received_at,
            byte_length=len(data),
            header=header,
            subheader=subheader,
            valid=valid,
            transport_notifications=self._transport_notifications,
            valid_packets=self._valid_packets + (1 if valid else 0),
            invalid_packets=self._invalid_packets + (0 if valid else 1),
        )
        self._last_transport = info
        self._emit('transport', info)
        if packet is None:
            self._invalid_packets += 1
            signature = (len(data), header, subheader)
            if signature not in self._reported_invalid:
                self._reported_invalid.add(signature)
                self._emit_diagnostic('invalid-packet', byte_length=len(data), header=header, subheader=subheader)
            return

        self._valid_packets += 1
        self._last_valid_packet_at = received_at
        if self._valid_packets == 1:
            self._emit_diagnostic('first-valid-packet', byte_length=len(data), header=header,
                                  subheader=subheader, packet_type=packet.type)
        self._resolve_packet_waiters(True)
        self._emit('raw', packet)
        if isinstance(packet, GaitMotionPacket):
            self._emit('motion', packet)
            return
        row = self.aggregator.add(packet)
        if row is not None:
            self.rows.append(row)
            self._emit('gait', row)

    def _attach_aggregator(self) -> None:
        """集計器の欠けの知らせを step_loss / diagnostic('step-loss') につなぐ（集計器を作り直すたびに呼ぶ）"""
        def on_step_loss(info: GaitStepLossInfo) -> None:
            self._emit('step_loss', info)
            self._emit_diagnostic('step-loss', info=info)

        self.aggregator.on_step_loss = on_step_loss

    def _resolve_packet_waiters(self, arrived: bool) -> None:
        for waiter in list(self._packet_waiters):
            after, future = waiter
            if arrived and self._valid_packets <= after:
                continue
            self._packet_waiters.remove(waiter)
            if not future.done():
                future.set_result(arrived)

    # ── CSV ───────────────────────────────────────────────────

    def to_csv(self) -> str:
        """これまでの rows をヘッダ付きの CSV 文字列にする"""
        return '\n'.join([GAIT_CSV_HEADER, *(gait_row_to_csv(row) for row in self.rows)]) + '\n'

    def save_csv(self, path: str) -> None:
        with open(path, 'w', newline='') as f:
            f.write(self.to_csv())

