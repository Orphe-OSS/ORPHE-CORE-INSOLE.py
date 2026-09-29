"""
FifoRecorder — FIFO（ところてん）でロスレス収録する

    fifo = FifoRecorder(device)          # connect() 済みの OrpheCoreInsole
    fifo.on('samples', lambda samples: ...)
    fifo.on('data_loss', lambda info: print(info))
    if await fifo.start():
        await asyncio.sleep(60)
        await fifo.stop()
        fifo.save_csv('capture.csv')

収録中は SENSOR_VALUES の notify を横取りするので、リアルタイムのイベント（acc など）は止まる。
stop() すると FW をリアルタイム通信に戻す（INSOLE は直前の streaming mode、CORE は 0x01）。
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Literal

from ..firmware import CORE_FIFO_MIN_RELEASE_DATE, INSOLE_FIFO_MIN_RELEASE_DATE, supports_fifo
from ..profiles.base import imu_range_from_settings
from .protocol import (
    CORE_REALTIME_READ_MODE,
    DEFAULT_DRAIN_TIMEOUT_SEC,
    FIFO_READ_MODE,
    FIFO_REQUEST_FULL_SIZE,
    INSOLE_DEFAULT_STREAMING_MODE,
    MAX_DATA_NUMBER_REQUESTED_AT_ONCE,
    OP_INFO,
    OP_READ_MODE,
    RESP_STATUS,
    SUB_DELETE_ALL,
    SUB_GET_SERIAL,
    SUB_START_MONITOR,
    SUB_STOP_MONITOR,
    UINT16_MAX,
    FifoCurrentSerial,
    FifoRequestRange,
    FifoSample,
    FifoScale,
    build_requests_from_serials,
    calc_expected_serials,
    create_get_sensor_data_request,
    decode_fifo_packet,
    expand_requests_to_list,
    extract_timestamp_ms,
    layout_of,
    max_pairs_per_request,
    parse_current_serial,
    parse_no_data_response,
    raw_store_to_csv,
    serial_distance,
)
from .state import DrainBudget, FifoLoopState, FifoLossEvent, FifoLossReason, NotifyQueue

if TYPE_CHECKING:
    from ..device import OrpheCoreInsole

logger = logging.getLogger(__name__)

FIFO_EVENTS = ('samples', 'progress', 'anomaly', 'data_loss', 'stopped', 'error', 'write', 'notify')

# restore_mode を指定しなかったことを表す（None は「復帰しない」の意味で使う）
_DEFAULT = object()


@dataclass
class FifoTiming:
    """ポーリングの間隔と待ち時間[s]"""

    polling_interval_sec: float = 0.2        # ポーリング間隔
    current_serial_timeout_sec: float = 0.05  # 現在シリアル番号の応答を待つ時間
    one_shot_timeout_sec: float = 5.0        # 1 回の要求の応答全体を待つ時間
    one_shot_idle_timeout_sec: float = 0.4   # 受信が始まってから許す無音
    command_ack_timeout_sec: float = 0.3     # ACK を待つ時間
    mode_switch_delay_sec: float = 0.1       # 読み取りモードを変えたあとの待ち
    fifo_mode_settle_sec: float = 0.2        # FIFO に切り替えてから、直前のリアルタイムパケットを捨てるまで
    retry_interval_sec: float = 1.0          # 準備コマンドを再試行する間隔


@dataclass(frozen=True)
class FifoProgressInfo:
    collected: int              # 回収したパケット数
    last_received: int          # 直近の要求で受け取ったパケット数
    current_serial: int | None  # FW の最新シリアル番号
    lag: int                    # 追従の遅れ（まだ取っていないシリアル数）。1500 に近づくと欠損する
    dropped: int                # 回復できなかった累計
    draining: bool = False      # stop() 後の回収中
    catchup: bool = False       # stop() 時点のバックログを回収中


@dataclass(frozen=True)
class FifoAnomalyInfo:
    """要求した件数と受け取った件数がずれたときの内訳"""

    start_serial: int
    request_size: int
    current_serial: int
    received: int
    expected: int
    no_data: int          # FW が no-data と返した件数
    ble_loss: int         # BLE で落ちた件数（次のループで再要求する）
    confirmed_lost: int   # 再要求しても FW に無く、失われた件数
    new_no_data: int      # 新しく要求した範囲の no-data


@dataclass(frozen=True)
class FifoDataLossInfo:
    reason: FifoLossReason
    dropped: int
    cumulative: int
    current_serial: int | None


@dataclass(frozen=True)
class FifoStoppedInfo:
    reason: Literal['manual', 'loss']  # loss は stop_on_loss による自動停止
    dropped: int
    collected: int
    drain_recovered: int      # stop() 後の再要求で回収した数
    catchup_recovered: int    # stop() 後のバックログ回収で回収した数


@dataclass
class _Received:
    packets: dict[int, bytes] = field(default_factory=dict)
    no_data: set[int] = field(default_factory=set)


class FifoRecorder:
    """
    FIFO（ロスレス）収録

    Args:
        device: connect() 済みの OrpheCoreInsole（CORE は FW 20260905 以降、INSOLE は 20260510 以降）
        startup_delay_sec: 蓄積を始めてから最初に要求するまでの待ち
        read_pressure_calibrations: INSOLE で個体別圧力校正係数をまだ読んでいなければ、開始前に読む（CSV の N 換算に使う）
        stop_on_loss: 回復できない欠損が出たら収録を止める
        drain_timeout_sec: stop() 後に取りこぼしを回収する時間（無音の許容時間。0 で回収しない）
        restore_mode: stop() 後に書き戻す読み取りモード。既定は INSOLE が直前の streaming mode
            （不明なら 4）、CORE が 1。None なら書き戻さない
        timing: ポーリングの間隔・待ち時間
    """

    def __init__(
        self,
        device: OrpheCoreInsole,
        *,
        startup_delay_sec: float = 1.0,
        read_pressure_calibrations: bool = True,
        stop_on_loss: bool = False,
        drain_timeout_sec: float = DEFAULT_DRAIN_TIMEOUT_SEC,
        restore_mode: Any = _DEFAULT,
        timing: FifoTiming | None = None,
        wait: Callable[[float], Awaitable[None]] | None = None,
    ) -> None:
        self.device = device
        self.startup_delay_sec = startup_delay_sec
        self.read_pressure_calibrations = read_pressure_calibrations
        self.stop_on_loss = stop_on_loss
        self.drain_timeout_sec = drain_timeout_sec
        self.timing = timing or FifoTiming()
        self.state = FifoLoopState()
        self.lag = 0
        self.scale = FifoScale()
        self._restore_mode_option = restore_mode
        self._restore_mode: int | None = None
        self._wait = wait or asyncio.sleep
        self._listeners: dict[str, list[Callable[..., Any]]] = {event: [] for event in FIFO_EVENTS}
        self._queue = NotifyQueue()
        self._running = False
        self._starting = False
        self._loop_task: asyncio.Task[None] | None = None
        self._remove_sink: Callable[[], None] | None = None
        self._started_notify = False
        self._torn_down = False
        self._auto_stopped = False
        self._last_current_serial: int | None = None
        # データ要求 1 回のバイト数（start() で書き込める長さから決める）と載せられる組数
        self.request_size = FIFO_REQUEST_FULL_SIZE
        self._max_pairs = max_pairs_per_request(FIFO_REQUEST_FULL_SIZE)

    # ── 状態 ──────────────────────────────────────────────────

    @property
    def kind(self) -> str:
        return self.device.kind

    @property
    def collected_count(self) -> int:
        """回収したパケット数"""
        return len(self.state.raw_store)

    @property
    def dropped_count(self) -> int:
        """回復できなかった累計シリアル数（0 なら欠損なし）"""
        return self.state.dropped

    @property
    def is_running(self) -> bool:
        return self._running

    def reset(self) -> None:
        """回収したデータと欠損の数を捨てる（収録中は何もしない）"""
        if self._running:
            return
        self.state = FifoLoopState()
        self.lag = 0
        self._last_current_serial = None

    # ── イベント ──────────────────────────────────────────────

    def on(self, event: str, callback: Callable[..., Any]) -> Callable[[], None]:
        """
        イベントを購読する。戻り値を呼ぶと購読をやめる

        samples(list[FifoSample])  回収したパケットのサンプル（古い順）
        progress(FifoProgressInfo) 要求ごとの進み具合
        anomaly(FifoAnomalyInfo)   要求と受信の件数がずれたとき
        data_loss(FifoDataLossInfo) 回復できない欠損が確定したとき
        stopped(FifoStoppedInfo)   収録が終わったとき
        error(Exception)           収録中の例外
        write(bytes) / notify(bytes) FW へ書いたコマンドと受け取った notify（調べるとき用）
        """
        if event not in self._listeners:
            raise ValueError(f"unknown event: {event!r} (choose from {', '.join(FIFO_EVENTS)})")
        self._listeners[event].append(callback)
        return lambda: self._listeners[event].remove(callback) if callback in self._listeners[event] else None

    def _emit(self, event: str, *args: Any) -> None:
        for callback in list(self._listeners[event]):
            try:
                callback(*args)
            except Exception as error:
                if event == 'error':
                    logger.exception("FIFO error listener failed")
                else:
                    self._report_error(error)

    def _report_error(self, error: BaseException) -> None:
        if self._listeners['error']:
            self._emit('error', error)
        else:
            logger.warning("FIFO: %s", error)

    # ── 低レベルのコマンド ─────────────────────────────────────

    async def _write(self, data: bytes | bytearray | list[int]) -> None:
        data = bytes(data)
        self._emit('write', data)
        await self.device.write_command(data)

    def _request(self, pairs: list[FifoRequestRange]) -> bytes:
        return create_get_sensor_data_request(pairs, self.request_size)

    def _on_notify(self, data: bytes) -> None:
        if self._listeners['notify']:
            self._emit('notify', data)
        self._queue.push(data)

    async def _set_read_mode(self, mode: int) -> None:
        await self._write([OP_READ_MODE, mode])
        await self._wait(self.timing.mode_switch_delay_sec)

    async def _command_expect_ack(self, sub: int) -> bool:
        self._queue.drain()
        await self._write([OP_INFO, sub])
        data = await self._queue.wait(self.timing.command_ack_timeout_sec)
        return data is not None and len(data) >= 2 and data[0] == RESP_STATUS and data[1] == sub

    async def _get_current_serial(self) -> FifoCurrentSerial | None:
        self._queue.drain()
        await self._write([OP_INFO, SUB_GET_SERIAL])
        data = await self._queue.wait(self.timing.current_serial_timeout_sec)
        return parse_current_serial(data) if data is not None else None

    async def _retry(self, fn: Callable[[], Awaitable[bool]], retries: int = 10) -> bool:
        for _ in range(retries):
            if await fn():
                return True
            await self._wait(self.timing.retry_interval_sec)
        return False

    async def _retry_current_serial(self, retries: int = 10) -> FifoCurrentSerial | None:
        for _ in range(retries):
            current = await self._get_current_serial()
            if current is not None:
                return current
            await self._wait(0.001)
        return None

    # ── 開始 ──────────────────────────────────────────────────

    async def start(self) -> bool:
        """
        FIFO 収録を始める

        Returns:
            始められたら True。未接続・FW が未対応・準備コマンドに応答が無いときは False（error イベントで理由を通知）
        """
        if self._running or self._starting:
            return self._running
        # 直前の収録の後片付けが終わるのを待つ（旧ループと notify を奪い合わないため）
        self._starting = True
        try:
            if self._loop_task is not None:
                await asyncio.gather(self._loop_task, return_exceptions=True)
        finally:
            self._starting = False

        device = self.device
        if not device.is_connected():
            self._report_error(RuntimeError("FifoRecorder.start(): device is not connected"))
            return False
        if self.kind not in ('core', 'insole'):
            self._report_error(RuntimeError("FifoRecorder.start(): device kind is not determined"))
            return False

        firmware = device.firmware or await device.read_firmware_info()
        if supports_fifo(self.kind, firmware) is False:
            minimum = CORE_FIFO_MIN_RELEASE_DATE if self.kind == 'core' else INSOLE_FIFO_MIN_RELEASE_DATE
            self._report_error(RuntimeError(
                f"FIFO is not available on firmware {firmware.release_date} (requires {minimum} or later)"))
            return False

        self.state = FifoLoopState()
        self.lag = 0
        # INSOLE は CSV の圧力換算に個体別校正係数を使う。まだ読んでいなければ FIFO の前に読む
        if self.kind == 'insole' and self.read_pressure_calibrations and device.pressure_calibrations is None:
            try:
                await device.read_pressure_calibrations()
            except Exception as error:  # noqa: BLE001 (読めなくても固定式で収録は続ける)
                self._report_error(error)

        if self._restore_mode_option is not _DEFAULT:
            self._restore_mode = self._restore_mode_option
        elif self.kind == 'core':
            self._restore_mode = CORE_REALTIME_READ_MODE
        else:
            self._restore_mode = device.streaming_mode or INSOLE_DEFAULT_STREAMING_MODE

        # 1 回で書き込める長さが 122 バイトに足りなければ、データ要求を短くして送る
        max_write = device.max_write_size()
        self.request_size = min(FIFO_REQUEST_FULL_SIZE, max_write) if max_write else FIFO_REQUEST_FULL_SIZE
        self._max_pairs = max_pairs_per_request(self.request_size)

        # FIFO の生値は device information のレンジで換算する
        di = await device.read_device_information()
        self.scale = FifoScale(*imu_range_from_settings(di.range))

        try:
            self._remove_sink = device.set_notify_sink(self._on_notify)
        except RuntimeError as error:
            self._report_error(error)
            return False

        try:
            if not device.sensor_values_notifying:
                await device.start_sensor_values_notification()
                self._started_notify = True
            await self._set_read_mode(FIFO_READ_MODE)
            await self._wait(self.timing.fifo_mode_settle_sec)
            self._queue.drain()  # 切り替え前のリアルタイムパケットを捨てる
            if not await self._prepare():
                self._report_error(RuntimeError(
                    "FifoRecorder.start(): the device did not respond to FIFO commands"))
                await self._teardown()
                return False
        except Exception as error:  # noqa: BLE001 (失敗したら後片付けして False を返す)
            self._report_error(error)
            await self._teardown()
            return False

        self._running = True
        self._torn_down = False
        self._auto_stopped = False
        self._loop_task = asyncio.get_running_loop().create_task(self._run_loop_wrapped())
        return True

    async def _prepare(self) -> bool:
        """蓄積を止め、全消去して、蓄積を始める"""
        for sub in (SUB_STOP_MONITOR, SUB_DELETE_ALL, SUB_START_MONITOR):
            if not await self._retry(lambda sub=sub: self._command_expect_ack(sub)):
                return False
        if self.startup_delay_sec > 0:
            await self._wait(self.startup_delay_sec)
        return True

    # ── 停止 ──────────────────────────────────────────────────

    async def stop(self) -> dict[int, bytes]:
        """
        収録を止めてリアルタイム通信に戻す。取りこぼしの回収（最大 drain_timeout_sec の無音まで）を待つ

        Returns:
            回収したパケット（シリアル番号 → 生データ）
        """
        if self._loop_task is None:
            return self.state.raw_store
        self._running = False
        await asyncio.gather(self._loop_task, return_exceptions=True)
        self._loop_task = None
        return self.state.raw_store

    async def _run_loop_wrapped(self) -> None:
        drain_recovered = 0
        catchup_recovered = 0
        try:
            await self._run_loop()
            # 手動 stop() のときだけ回収フェーズを走らせる:
            #   catch-up  stop() 時点で FW に溜まっていて、まだ要求していないバックログを回収する
            #   drain     再要求キューに残った取りこぼしを回収する
            if not self._auto_stopped and self.drain_timeout_sec > 0 and self.device.is_connected():
                budget = DrainBudget(self.drain_timeout_sec)
                catchup_recovered = await self._catch_up_loop(await self._resolve_catch_up_target(), budget)
                drain_recovered = await self._drain_loop(budget)
        except Exception as error:  # noqa: BLE001 (ループの失敗を通知して必ず後片付けする)
            self._report_error(error)
        finally:
            # 回収できないまま終わったぶんを必ず dropped に入れる（dropped == 0 なら CSV は欠損なし）
            pending = self.state.finalize_pending_loss()
            if pending > 0:
                self._emit('data_loss', FifoDataLossInfo(
                    'stopped_pending', pending, self.state.dropped, self._last_current_serial))
            self.state.loss_events.clear()
            await self._teardown_once()
            self._running = False
            self._emit('stopped', FifoStoppedInfo(
                'loss' if self._auto_stopped else 'manual', self.state.dropped,
                len(self.state.raw_store), drain_recovered, catchup_recovered))

    async def _teardown_once(self) -> None:
        if self._torn_down:
            return
        self._torn_down = True
        await self._teardown()

    async def _teardown(self) -> None:
        """蓄積を止め、読み取りモードを戻し、notify の横取りをやめる"""
        try:
            if self.device.is_connected():
                # 蓄積の停止は FW 側の収録を止める要なので、ACK が返るまで再試行する
                try:
                    await self._retry(lambda: self._command_expect_ack(SUB_STOP_MONITOR), 3)
                except Exception:  # noqa: BLE001, S110 (後片付けは続ける)
                    pass
                if self._restore_mode is not None:
                    try:
                        await self._set_read_mode(self._restore_mode)
                        if self.kind == 'insole':
                            self.device.streaming_mode = self._restore_mode
                    except Exception:  # noqa: BLE001, S110
                        pass
                if self._started_notify:
                    try:
                        await self.device.stop_sensor_values_notification()
                    except Exception:  # noqa: BLE001, S110
                        pass
        finally:
            self._started_notify = False
            if self._remove_sink is not None:
                self._remove_sink()
                self._remove_sink = None

    # ── メインループ ───────────────────────────────────────────

    async def _run_loop(self) -> None:
        state = self.state
        while self._running and self.device.is_connected():
            current = await self._retry_current_serial()
            if not self._running:
                break
            if current is None:
                await self._wait(self.timing.polling_interval_sec)
                continue

            current_serial = current.serial
            self._last_current_serial = current_serial
            self.lag = (current.accumulated if state.last_serial is None
                        else serial_distance(state.last_serial, current_serial))

            carry_over = state.carry_over[:self._max_pairs]
            carry_over_count = sum(count for _, count in carry_over)
            max_new_request = max(0, MAX_DATA_NUMBER_REQUESTED_AT_ONCE - carry_over_count)
            start_serial, request_size = state.calc_request_range(
                current_serial, current.accumulated, max_new_request)

            if request_size <= 0 and not carry_over:
                await self._wait(self.timing.polling_interval_sec)
                continue
            # 新しい範囲も送るときは、1 回に載せられる組数に収まるよう再要求を 1 組減らす
            if request_size > 0 and len(carry_over) >= self._max_pairs:
                carry_over = carry_over[:self._max_pairs - 1]

            requests: list[FifoRequestRange] = ([(start_serial, request_size)] if request_size > 0 else [])
            requests += carry_over
            state.carry_over = state.carry_over[len(carry_over):]

            expected = set(expand_requests_to_list(requests))
            new_serials = set(calc_expected_serials(start_serial, request_size))

            self._queue.drain()
            await self._write(self._request(requests))
            received = await self._receive_responses(
                expected, self.timing.one_shot_timeout_sec, self.timing.one_shot_idle_timeout_sec)
            # stop() で running が落ちても、この回に受け取ったぶんは格納する（未受信ぶんは drain で回収）

            missed = [serial for serial in expected if serial not in received.packets]
            ble_loss = {serial for serial in missed if serial not in received.no_data}
            confirmed_lost = [serial for serial in missed if serial in received.no_data]
            new_no_data = {serial for serial in received.no_data if serial in new_serials}

            if confirmed_lost:
                state.dropped += len(confirmed_lost)
                state.loss_events.append(FifoLossEvent('fw_nodata', len(confirmed_lost)))
            if received.no_data or missed:
                self._emit('anomaly', FifoAnomalyInfo(
                    start_serial, request_size, current_serial, len(received.packets), len(expected),
                    len(received.no_data), len(ble_loss), len(confirmed_lost), len(new_no_data)))

            state.update_after_response(ble_loss, new_no_data, start_serial, request_size)

            if state.loss_events:
                self._flush_loss_events(current_serial)
                if self.stop_on_loss:
                    self._auto_stopped = True
                    self._running = False

            self._absorb(received.packets, overwrite=True)
            self._emit('progress', FifoProgressInfo(
                len(state.raw_store), len(received.packets), current_serial, self.lag, state.dropped))
            await self._wait(self.timing.polling_interval_sec)

    async def _receive_responses(self, expected: set[int], timeout_sec: float, idle_timeout_sec: float) -> _Received:
        """
        期限まで notify を受け取り、データパケットと no-data に分ける

        受信が始まったあと idle_timeout_sec 無音が続いたら、そのバーストは終わりとみなす
        （取りこぼしは再要求で回収するので問題ない）。
        """
        result = _Received()
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_sec
        got_any = False
        while len(result.packets) + len(result.no_data) < len(expected):
            remaining = deadline - loop.time()
            if remaining <= 0:
                break
            data = await self._queue.wait(min(remaining, idle_timeout_sec) if got_any else remaining)
            if data is None:
                break
            got_any = True
            no_data = parse_no_data_response(data)
            if no_data is not None:
                start, count = no_data
                result.no_data.update((start + i) % UINT16_MAX for i in range(count))
                continue
            if layout_of(data, self.kind) is not None:
                serial = int.from_bytes(data[1:3], 'big')
                if serial in expected:
                    result.packets[serial] = data
        return result

    def _absorb(self, packets: dict[int, bytes], overwrite: bool = False) -> int:
        """受け取ったパケットを格納して samples を通知する。新しく格納した数を返す"""
        state = self.state
        samples: list[FifoSample] = []
        stored = 0
        for serial, data in packets.items():
            if serial in state.raw_store and not overwrite:
                continue
            state.raw_store[serial] = data
            state.note_stored(serial)
            stored += 1
            layout = layout_of(data, self.kind)
            if layout is not None:
                samples += decode_fifo_packet(data, layout, self.scale)
        if samples:
            self._emit('samples', samples)
        return stored

    def _flush_loss_events(self, current_serial: int | None) -> None:
        events, self.state.loss_events = self.state.loss_events, []
        for event in events:
            self._emit('data_loss', FifoDataLossInfo(event.reason, event.dropped, self.state.dropped, current_serial))

    def _classify_missed(self, expected: set[int], received: _Received) -> None:
        """受け取れなかったシリアルを、再要求するもの（BLE で落ちた）と失われたもの（FW に無い）に分ける"""
        missed = [serial for serial in expected if serial not in received.packets]
        confirmed_lost = [serial for serial in missed if serial in received.no_data]
        ble_loss = {serial for serial in missed if serial not in received.no_data}
        if confirmed_lost:
            self.state.dropped += len(confirmed_lost)
            self.state.loss_events.append(FifoLossEvent('fw_nodata', len(confirmed_lost)))
        if ble_loss:
            self.state.carry_over += build_requests_from_serials(ble_loss)

    # ── stop() 後の回収 ───────────────────────────────────────

    async def _resolve_catch_up_target(self) -> int | None:
        """stop() 時点の FW の書き込み位置を 1 回だけ取って固定する（停止後に増えるぶんは追わない）"""
        try:
            current = await self._retry_current_serial(3)
            if current is not None:
                self._last_current_serial = current.serial
                return current.serial
        except Exception:  # noqa: BLE001, S110 (取れなければ最後のポーリング値を使う)
            pass
        return self._last_current_serial

    async def _catch_up_loop(self, target: int | None, budget: DrainBudget) -> int:
        """まだ要求していないバックログを target まで前から順に回収する"""
        state = self.state
        if target is None or state.last_serial is None:
            return 0
        backlog = serial_distance(state.last_serial, target)
        if backlog == 0 or backlog > UINT16_MAX / 2:
            return 0
        # 回収しきれなかった末尾も stopped_pending として数えられるよう、収録範囲を target まで延ばす
        state.note_span_target(target)

        recovered = 0
        while not budget.expired:
            anchor = state.last_serial
            remaining = serial_distance(anchor, target)
            if remaining <= 0:
                break
            request_size = min(remaining, MAX_DATA_NUMBER_REQUESTED_AT_ONCE)
            start_serial = (anchor + 1) % UINT16_MAX
            expected = set(calc_expected_serials(start_serial, request_size))

            self._queue.drain()
            await self._write(self._request([(start_serial, request_size)]))
            shot_timeout = min(self.timing.one_shot_timeout_sec, budget.remaining_sec())
            if shot_timeout <= 0:
                break
            received = await self._receive_responses(expected, shot_timeout, self.timing.one_shot_idle_timeout_sec)

            # target は固定なので、要求済みの境界は無条件に進める
            state.last_serial = (start_serial + request_size - 1) % UINT16_MAX
            self._classify_missed(expected, received)
            stored = self._absorb(received.packets)
            recovered += stored
            if stored > 0:
                budget.note_progress()
            self._flush_loss_events(self._last_current_serial)
            self._emit('progress', FifoProgressInfo(
                len(state.raw_store), len(received.packets), self._last_current_serial,
                serial_distance(state.last_serial, target) + state.carry_over_count(), state.dropped,
                draining=True, catchup=True))
        return recovered

    async def _drain_loop(self, budget: DrainBudget) -> int:
        """再要求キューに残った取りこぼしを回収する。FW から消えたぶんは fw_nodata として確定する"""
        state = self.state
        recovered = 0
        while state.carry_over and not budget.expired:
            to_send = state.carry_over[:self._max_pairs]
            state.carry_over = state.carry_over[len(to_send):]
            expected = set(expand_requests_to_list(to_send))
            if not expected:
                continue

            self._queue.drain()
            await self._write(self._request(to_send))
            shot_timeout = min(self.timing.one_shot_timeout_sec, budget.remaining_sec())
            received = await self._receive_responses(expected, shot_timeout, self.timing.one_shot_idle_timeout_sec)

            self._classify_missed(expected, received)
            stored = self._absorb(received.packets)
            recovered += stored
            if stored > 0:
                budget.note_progress()
            self._flush_loss_events(self._last_current_serial)
            self._emit('progress', FifoProgressInfo(
                len(state.raw_store), len(received.packets), self._last_current_serial,
                state.carry_over_count(), state.dropped, draining=True))
        return recovered

    # ── 出力 ──────────────────────────────────────────────────

    def samples(self) -> list[FifoSample]:
        """回収したサンプルを時刻順に並べて返す"""
        out: list[FifoSample] = []
        for data in sorted(self.state.raw_store.values(), key=extract_timestamp_ms):
            layout = layout_of(data, self.kind)
            if layout is not None:
                out += decode_fifo_packet(data, layout, self.scale)
        return out

    def to_csv(self) -> str:
        """回収したデータを CSV にする（時刻順。INSOLE の圧力は個体別校正係数、無ければ固定式で N に換算）"""
        calibrations = self.device.pressure_calibrations if self.kind == 'insole' else None
        return raw_store_to_csv(self.state.raw_store, self.kind, self.scale, calibrations)

    def save_csv(self, path: str) -> None:
        with open(path, 'w', newline='') as f:
            f.write(self.to_csv())
