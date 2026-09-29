"""
orphe-core-insole コマンド

    orphe-core-insole scan                     周りの ORPHE CORE / INSOLE を探す
    orphe-core-insole info                     device information を表示する
    orphe-core-insole set --acc 16 --gyro 2000 設定を書き込む
    orphe-core-insole stream --csv out.csv     センサー値を受信する（受信レートと欠損を毎秒表示）
    orphe-core-insole step                     歩行解析を表示する
    orphe-core-insole check                    実機で主な機能を確かめて PASS / FAIL を出す

接続の経過や結果の要約は標準エラー、データ（--print）は標準出力に出す。
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, TextIO

from . import __version__
from .device import OrpheCoreInsole, UnsupportedOperationError
from .fifo import FifoRecorder
from .fifo.protocol import FIFO_IMU_ODR_HZ, extract_timestamp_ms, layout_of
from .firmware import CORE_FIFO_MIN_RELEASE_DATE, INSOLE_FIFO_MIN_RELEASE_DATE, supports_fifo
from .gait import InsoleGait
from .packets import SensorFrame, SensorValuesPacket
from .profiles import ACC_RANGES, GYRO_RANGES, AutoProfile, core_profile, insole_profile

# (デバイス種別, SENSOR_VALUES のヘッダ) ごとの想定サンプリングレート[Hz]
EXPECTED_RATES = {
    ('core', 40): 50,
    ('core', 50): 200,
    ('insole', 50): 200,
    ('insole', 55): 200,
    ('insole', 56): 100,
}
# INSOLE の streaming mode と届くヘッダ
INSOLE_MODE_HEADERS = {1: 50, 3: 55, 4: 56}
DEFAULT_INSOLE_MODE = 4
# SENSOR_VALUES のシリアル番号は 16 ビット
SERIAL_NUMBER_MODULO = 65536

CSV_COLUMNS = [
    'timestamp', 'serial_number', 'packet_number',
    'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z',
    'quat_w', 'quat_x', 'quat_y', 'quat_z',
    'pressure_1', 'pressure_2', 'pressure_3', 'pressure_4', 'pressure_5', 'pressure_6',
    'pressure_1[N]', 'pressure_2[N]', 'pressure_3[N]', 'pressure_4[N]', 'pressure_5[N]', 'pressure_6[N]',
]


def log(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def create_device(args: argparse.Namespace) -> OrpheCoreInsole:
    """--device に合わせて OrpheCoreInsole を作る（テストで差し替える）"""
    profile = {'core': core_profile, 'insole': insole_profile}.get(args.device)
    return OrpheCoreInsole(profile() if profile else AutoProfile(), verbose=False)


async def connect(args: argparse.Namespace) -> OrpheCoreInsole | None:
    device = create_device(args)
    target = args.address or {'core': 'ORPHE CORE', 'insole': 'ORPHE INSOLE'}.get(args.device, 'ORPHE CORE / INSOLE')
    log(f"スキャン中... ({target})")
    if not await device.connect(args.address, timeout=args.timeout):
        log("デバイスが見つかりません")
        return None
    log(f"接続しました: {device.name} ({device.address}) kind={device.kind}")
    return device


async def close(device: OrpheCoreInsole) -> None:
    if device.is_connected():
        await device.disconnect()
        log("切断しました")


def unwrap(value: Any) -> Any:
    """struct.unpack の 1 要素タプルを中身にする（表示用）"""
    return value[0] if isinstance(value, tuple) and len(value) == 1 else value


def frame_row(frame: SensorFrame, device: OrpheCoreInsole) -> list[Any]:
    """CSV の 1 行。加速度は [G]、角速度は [deg/s]、圧力は ADC 値と [N]。無い値は空欄"""
    acc, gyro, quat, pressure = frame.converted_acc, frame.converted_gyro, frame.quat, frame.pressure
    return [
        acc.timestamp, acc.serial_number, acc.packet_number,
        acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z,
        *((quat.w, quat.x, quat.y, quat.z) if quat else ('',) * 4),
        *(pressure.values if pressure else ('',) * 6),
        *(device.convert_pressure(pressure).values if pressure else ('',) * 6),
    ]


def describe_calibrations(calibrations: list | None) -> list[str]:
    """read_pressure_calibrations() の結果を表示用の行にする"""
    if calibrations is None:
        return ["非対応 FW（固定式で換算）"]
    lines = []
    for index, calibration in enumerate(calibrations):
        if calibration is None:
            lines.append(f"ch{index + 1}: なし（固定式で換算）")
        else:
            coefficients = ', '.join(f"{c:.6g}" for c in calibration.coefficients)
            lines.append(f"ch{index + 1}: func={calibration.func} ({coefficients})")
    return lines


async def read_calibrations(device: OrpheCoreInsole) -> None:
    """INSOLE なら個体別圧力校正係数を読んで、読めた ch 数を表示する"""
    if device.kind != 'insole':
        return
    calibrations = await device.read_pressure_calibrations()
    if calibrations is None:
        log("圧力校正係数: 非対応 FW のため固定式で換算します")
    else:
        count = sum(1 for calibration in calibrations if calibration is not None)
        log(f"圧力校正係数: {count}/6 ch（残りは固定式で換算）")


def format_frame(frame: SensorFrame, device: OrpheCoreInsole) -> str:
    acc, gyro = frame.converted_acc, frame.converted_gyro
    text = (f"{acc.timestamp} [{acc.serial_number}:{acc.packet_number}] "
            f"acc=({acc.x:.3f}, {acc.y:.3f}, {acc.z:.3f}) gyro=({gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f})")
    if frame.quat:
        q = frame.quat
        text += f" quat=({q.w:.4f}, {q.x:.4f}, {q.y:.4f}, {q.z:.4f})"
    if frame.pressure:
        newton = ', '.join(f"{v:.1f}" for v in device.convert_pressure(frame.pressure).values)
        text += f" pressure={frame.pressure.values} [{newton}]N"
    return text


@dataclass
class StreamStats:
    """
    受信したサンプル数・欠損・静止時の値の集計

    欠損は受け取ったパケットのシリアル番号の飛びから数える。SDK の lost_data は公開版と同じく
    直前のシリアル番号の初期値が 0 で、接続直後の最初のパケットも欠損として数えてしまうため使わない。
    """

    # 直前に受け取ったパケットのシリアル番号（まだ無ければ None）
    prev_serial: int | None = None
    packets: int = 0
    samples: int = 0
    lost_packets: int = 0
    headers: set[int] = field(default_factory=set)
    first_at: float | None = None
    first_samples: int = 0
    last_at: float | None = None
    acc_norms: list[float] = field(default_factory=list)
    gyro_abs: list[float] = field(default_factory=list)
    quat_norms: list[float] = field(default_factory=list)
    pressure_samples: int = 0

    def add_packet(self, packet: SensorValuesPacket) -> None:
        now = time.monotonic()
        if self.first_at is None:
            self.first_at = now
            self.first_samples = len(packet.frames)
        self.last_at = now
        self.packets += 1
        self.samples += len(packet.frames)
        self.headers.add(packet.header)
        if packet.serial_number is not None:
            if self.prev_serial is not None:
                # 65535 の次は 0 に戻る
                self.lost_packets += (packet.serial_number - self.prev_serial - 1) % SERIAL_NUMBER_MODULO
            self.prev_serial = packet.serial_number
        for frame in packet.frames:
            acc, gyro = frame.converted_acc, frame.converted_gyro
            self.acc_norms.append(math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2))
            self.gyro_abs.append(max(abs(gyro.x), abs(gyro.y), abs(gyro.z)))
            if frame.quat:
                q = frame.quat
                self.quat_norms.append(math.sqrt(q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2))
            if frame.pressure and len(frame.pressure.values) == 6:
                self.pressure_samples += 1

    @property
    def rate(self) -> float:
        """最初のパケットから最後のパケットまでのサンプル数/秒"""
        if self.first_at is None or self.last_at is None or self.last_at <= self.first_at:
            return 0.0
        return (self.samples - self.first_samples) / (self.last_at - self.first_at)

    @property
    def loss_ratio(self) -> float:
        total = self.packets + self.lost_packets
        return self.lost_packets / total if total else 0.0


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else float('nan')


# ── scan / info / set ──────────────────────────────────────────

async def cmd_scan(args: argparse.Namespace) -> int:
    device = create_device(args)
    log(f"スキャン中...（{args.timeout:g} 秒）")
    results = await device.scan(timeout=args.timeout, all_devices=args.all)
    if not results:
        log("見つかりませんでした")
        return 1
    for result in sorted(results, key=lambda r: -(r.rssi or -999)):
        print(f"{result.kind or '-':6} {result.rssi!s:>4} dBm  {result.address}  {result.name}")
    return 0


async def cmd_info(args: argparse.Namespace) -> int:
    device = await connect(args)
    if device is None:
        return 1
    try:
        print(f"Name: {device.name}")
        print(f"Address: {device.address}")
        print(f"Kind: {device.kind}")
        firmware = await device.read_firmware_info()
        print(f"Firmware: {firmware.name if firmware else '-'}")
        await device.print_device_information()
        if device.kind == 'insole':
            print("Pressure calibration:")
            for line in describe_calibrations(await device.read_pressure_calibrations()):
                print(f"  {line}")
    finally:
        await close(device)
    return 0


async def cmd_set(args: argparse.Namespace) -> int:
    requested = [args.acc, args.gyro, args.lr, args.led_brightness, args.led, args.mount, args.mode]
    if all(value is None for value in requested):
        log("設定する項目を指定してください（orphe-core-insole set --help）")
        return 2
    device = await connect(args)
    if device is None:
        return 1
    try:
        if args.acc is not None:
            await device.set_acc_range(args.acc)
        if args.gyro is not None:
            await device.set_gyro_range(args.gyro)
        if args.lr is not None:
            await device.set_lr(args.lr)
        if args.led_brightness is not None:
            await device.set_led_brightness(args.led_brightness)
        if args.led is not None:
            await device.set_led(*args.led)
        if args.mount is not None:
            await device.set_mount_position(args.mount)
        if args.mode is not None:
            await device.set_data_streaming_mode(args.mode)
        await device.print_device_information()
    except (ValueError, UnsupportedOperationError) as error:
        log(f"エラー: {error}")
        return 2
    finally:
        await close(device)
    return 0


# ── stream / step ──────────────────────────────────────────────

async def wait_while_connected(device: OrpheCoreInsole, duration: float | None,
                               tick: Callable[[], None] | None = None, interval: float = 1.0) -> None:
    """duration 秒（None なら切断か Ctrl+C まで）待つ。interval ごとに tick を呼ぶ"""
    deadline = None if duration is None else time.monotonic() + duration
    while device.is_connected():
        remaining = interval if deadline is None else min(interval, deadline - time.monotonic())
        if remaining <= 0:
            break
        await asyncio.sleep(remaining)
        if tick:
            tick()


async def cmd_stream(args: argparse.Namespace) -> int:
    device = await connect(args)
    if device is None:
        return 1
    csv_file: TextIO | None = None
    total = StreamStats()
    window = StreamStats()
    try:
        if device.kind == 'insole':
            await read_calibrations(device)
            await device.set_data_streaming_mode(args.mode or DEFAULT_INSOLE_MODE)
        elif args.mode is not None:
            log("エラー: --mode は ORPHE INSOLE だけで使えます")
            return 2
        if args.acc is not None:
            await device.set_acc_range(args.acc)
        if args.gyro is not None:
            await device.set_gyro_range(args.gyro)

        writer = None
        if args.csv:
            csv_file = open(args.csv, 'w', newline='')  # noqa: ASYNC230, SIM115 (ローカルファイル。finally で閉じる)
            writer = csv.writer(csv_file)
            writer.writerow(CSV_COLUMNS)

        def on_packet(packet: SensorValuesPacket) -> None:
            total.add_packet(packet)
            window.add_packet(packet)
            for frame in packet.frames:
                if writer:
                    writer.writerow(frame_row(frame, device))
                if args.print:
                    print(format_frame(frame, device))

        def report() -> None:
            nonlocal window
            headers = ','.join(str(h) for h in sorted(window.headers)) or '-'
            log(f"{window.samples:5d} samples/s  lost {window.lost_packets} packets  header {headers}")
            # 1 秒ごとの区切りでも欠損を数えられるよう、直前のシリアル番号は引き継ぐ
            window = StreamStats(prev_serial=total.prev_serial)

        device.on('sensor_values', on_packet)
        await device.start_sensor_values_notification()
        log("受信中...（Ctrl+C で終了）" if args.duration is None else f"{args.duration:g} 秒受信します")
        await wait_while_connected(device, args.duration, tick=report)
        if device.is_connected():
            await device.stop_sensor_values_notification()
    finally:
        if csv_file:
            csv_file.close()
        await close(device)
        log(f"合計 {total.samples} samples（{total.rate:.1f} Hz）、欠損 {total.lost_packets} packets"
            f"（{total.loss_ratio:.2%}）" + (f"、CSV: {args.csv}" if args.csv else ''))
    return 0


def format_values(name: str, value: Any) -> str:
    return f"[{name}] " + ', '.join(f"{k}={unwrap(v)}" for k, v in vars(value).items())


async def cmd_step(args: argparse.Namespace) -> int:
    device = await connect(args)
    if device is None:
        return 1
    try:
        if device.kind == 'insole':
            return await step_insole(args, device)
        if args.csv:
            log("エラー: --csv は ORPHE INSOLE だけで使えます")
            return 2
        events = ['gait', 'stride', 'pronation'] + (['quat_distance'] if args.quat_distance else [])
        for event in events:
            device.on(event, lambda value, e=event: print(format_values(e, value)))
        await device.start_step_analysis_notification()
        log("受信中...（歩くと 1 歩ごとに表示されます。Ctrl+C で終了）")
        await wait_while_connected(device, args.duration)
        if device.is_connected():
            await device.stop_step_analysis_notification()
    finally:
        await close(device)
    return 0


async def step_insole(args: argparse.Namespace, device: OrpheCoreInsole) -> int:
    """INSOLE の歩行解析（1 歩ぶんそろうごとに表示する）"""
    gait = InsoleGait(device)
    gait.on('gait', lambda row: print(format_values('gait', row)))
    if args.quat_distance:
        gait.on('motion', lambda motion: print(format_values('motion', motion)))
    gait.on('error', lambda error: log(f"エラー: {error}"))
    if not await gait.start():
        return 1
    try:
        log("受信中...（歩くと 1 歩ごとに表示されます。Ctrl+C で終了）")
        await wait_while_connected(device, args.duration)
    finally:
        await gait.stop()
        loss = gait.diagnostics().step_loss
        log(f"合計 {gait.step_count} 歩、そろわなかった歩 {loss.incomplete_steps}、届かなかった歩 {loss.gap_steps}"
            + (f"、CSV: {args.csv}" if args.csv else ''))
        if args.csv:
            gait.save_csv(args.csv)
    return 0


# ── fifo ──────────────────────────────────────────────────────

def fifo_sample_rate(fifo: FifoRecorder) -> float:
    """回収したパケットの FW 時刻から求めたサンプル数/秒（パケットが 2 つ未満なら 0）"""
    packets = [(extract_timestamp_ms(data), layout.samples) for data in fifo.state.raw_store.values()
               if (layout := layout_of(data, fifo.kind)) is not None]
    if len(packets) < 2:
        return 0.0
    packets.sort()
    span_sec = (packets[-1][0] - packets[0][0]) / 1000
    samples = sum(count for _, count in packets[1:])
    return samples / span_sec if span_sec > 0 else 0.0


def describe_fifo_bytes(data: bytes) -> str:
    """FIFO のコマンド・応答を 1 行で表す（--debug 用）"""
    head = data[:12].hex(' ') + (' …' if len(data) > 12 else '')
    if len(data) >= 3 and data[0] == 0x36:
        return f"data serial={int.from_bytes(data[1:3], 'big')} ({len(data)}B)"
    return f"{head} ({len(data)}B)"


def fifo_requirement(kind: str) -> int:
    return CORE_FIFO_MIN_RELEASE_DATE if kind == 'core' else INSOLE_FIFO_MIN_RELEASE_DATE


async def cmd_fifo(args: argparse.Namespace) -> int:
    device = await connect(args)
    if device is None:
        return 1
    fifo = FifoRecorder(device, drain_timeout_sec=args.drain_timeout, stop_on_loss=args.stop_on_loss)
    window = {'samples': 0}
    errors: list[BaseException] = []

    def on_samples(samples: list) -> None:
        window['samples'] += len(samples)
        if args.print:
            for sample in samples:
                gyro, acc = sample.gyro, sample.acc
                text = (f"{sample.t:.1f} [{sample.serial_number}:{sample.packet_number}] "
                        f"acc=({acc[0]:.3f}, {acc[1]:.3f}, {acc[2]:.3f}) "
                        f"gyro=({gyro[0]:.2f}, {gyro[1]:.2f}, {gyro[2]:.2f})")
                print(text + (f" pressure={list(sample.pressure)}" if sample.pressure else ''))

    fifo.on('samples', on_samples)
    fifo.on('data_loss', lambda info: log(f"欠損: {info.reason} {info.dropped} packets（累計 {info.cumulative}）"))
    fifo.on('error', errors.append)
    if args.debug:
        fifo.on('error', lambda error: log(f"エラー: {type(error).__name__}: {error}"))
        fifo.on('write', lambda data: log(f"  → {describe_fifo_bytes(data)}"))
        fifo.on('notify', lambda data: log(f"  ← {describe_fifo_bytes(data)}"))
    try:
        firmware = await device.read_firmware_info()
        log(f"FW: {firmware.name if firmware else '不明'}")
        if args.debug:
            mtu = getattr(device.client, 'mtu_size', None)
            log(f"MTU: {mtu}  DEVICE_INFORMATION に 1 回で書ける長さ: {device.max_write_size()} bytes")
        if not await fifo.start():
            reason = errors[-1] if errors else '不明'
            log(f"FIFO を開始できませんでした: {reason}")
            if supports_fifo(device.kind, firmware) is None:
                log(f"FW のリリース日が {fifo_requirement(device.kind)} 以降か確認してください")
            return 1
        log("FIFO 収録中...（Ctrl+C で終了）" if args.duration is None else f"{args.duration:g} 秒収録します")
        if fifo.request_size < 122:
            log(f"データ要求は {fifo.request_size} バイト以下に縮めて送ります（1 回で書ける長さが 122 バイト未満のため）")

        def report() -> None:
            log(f"{window['samples']:5d} samples/s  collected {fifo.collected_count} packets  "
                f"lag {fifo.lag}  dropped {fifo.dropped_count}")
            window['samples'] = 0

        deadline = None if args.duration is None else time.monotonic() + args.duration
        while fifo.is_running and device.is_connected():
            remaining = 1.0 if deadline is None else min(1.0, deadline - time.monotonic())
            if remaining <= 0:
                break
            await asyncio.sleep(remaining)
            report()
    finally:
        # Ctrl+C で止めたときも、回収を終えてから CSV を書く
        if fifo.is_running:
            log("停止して取りこぼしを回収しています...")
            await fifo.stop()
        await close(device)
        if fifo.collected_count:
            if args.csv:
                fifo.save_csv(args.csv)
            log(f"回収 {fifo.collected_count} packets（{fifo_sample_rate(fifo):.1f} Hz）、欠損 {fifo.dropped_count}"
                + (f"、CSV: {args.csv}" if args.csv else ''))
    return 0 if fifo.dropped_count == 0 else 1


# ── check ─────────────────────────────────────────────────────

@dataclass
class Check:
    status: str  # PASS / FAIL / WARN / INFO
    name: str
    detail: str = ''


class CheckRunner:
    """実機確認の結果を集めながら表示する"""

    def __init__(self) -> None:
        self.results: list[Check] = []

    def add(self, status: str, name: str, detail: str = '') -> None:
        self.results.append(Check(status, name, detail))
        print(f"[{status}] {name}" + (f": {detail}" if detail else ''), flush=True)

    def expect(self, ok: bool, name: str, detail: str = '', otherwise: str = 'FAIL') -> bool:
        self.add('PASS' if ok else otherwise, name, detail)
        return ok

    def count(self, status: str) -> int:
        return sum(1 for result in self.results if result.status == status)


async def check_settings(runner: CheckRunner, device: OrpheCoreInsole, write: bool) -> None:
    """
    設定を別の値に書き込んで読み直し、元に戻す（write が True のときだけ）

    途中で止まっても（Ctrl+C・例外）元の値を書き戻す。
    """
    if not write:
        runner.add('INFO', '設定の書き込み', "確かめていません（--write を付けると、別の値を書き込んで元に戻します）")
        return
    di = await device.read_device_information()

    async def round_trip(name: str, current: Any, new: Any, write: Callable[[Any], Any],
                         read: Callable[[Any], Any]) -> None:
        await write(new)
        try:
            changed = read(await device.read_device_information())
        finally:
            await write(current)
        restored = read(await device.read_device_information())
        runner.expect(changed == new and restored == current, name,
                      f"{current} → {new}（読み直し {changed}）→ {current}（読み直し {restored}）")

    acc = di.range.acc
    await round_trip('加速度レンジの書き込み', ACC_RANGES[acc], ACC_RANGES[(acc + 1) % 4],
                     device.set_acc_range, lambda d: ACC_RANGES[d.range.acc])
    gyro = di.range.gyro
    await round_trip('角速度レンジの書き込み', GYRO_RANGES[gyro], GYRO_RANGES[(gyro + 1) % 4],
                     device.set_gyro_range, lambda d: GYRO_RANGES[d.range.gyro])

    if device.kind == 'core':
        await round_trip('左右（set_lr）の書き込み', di.lr, 0 if di.lr else 1, device.set_lr, lambda d: d.lr)
        await round_trip('LED の明るさの書き込み', di.led, 255 if di.led < 128 else 0,
                         device.set_led_brightness, lambda d: d.led)
        await device.set_led(1, 0)
        runner.add('INFO', 'LED 点灯コマンド', "送信しました。LED が点いているか目で確認してください")
    else:
        mount = di.mount_position
        await round_trip('マウント位置の書き込み', mount, mount ^ 1, device.set_mount_position,
                         lambda d: d.mount_position)


async def measure(device: OrpheCoreInsole, duration: float) -> StreamStats:
    stats = StreamStats()
    off = device.on('sensor_values', stats.add_packet)
    await device.start_sensor_values_notification()
    await wait_while_connected(device, duration, interval=min(duration, 0.5))
    await device.stop_sensor_values_notification()
    off()
    return stats


def check_stream(runner: CheckRunner, kind: str, label: str, stats: StreamStats,
                 expected_header: int | None) -> None:
    if not runner.expect(stats.packets > 0, f"{label} の受信", f"{stats.packets} packets"):
        return
    headers = sorted(stats.headers)
    if expected_header is not None:
        runner.expect(headers == [expected_header], f"{label} のヘッダ", f"{headers}（想定 {expected_header}）")
    else:
        runner.add('INFO', f"{label} のヘッダ", str(headers))

    expected_rate = EXPECTED_RATES.get((kind, headers[0]))
    if expected_rate:
        ratio = stats.rate / expected_rate
        status = 'PASS' if 0.8 <= ratio <= 1.2 else ('FAIL' if ratio < 0.5 else 'WARN')
        runner.add(status, f"{label} の受信レート", f"{stats.rate:.1f} Hz（想定 {expected_rate} Hz）")
    runner.expect(stats.lost_packets == 0, f"{label} の欠損",
                  f"{stats.lost_packets} packets（{stats.loss_ratio:.2%}）", otherwise='WARN')

    acc = mean(stats.acc_norms)
    runner.expect(0.9 <= acc <= 1.1, f"{label} の静止時の加速度", f"|acc| 平均 {acc:.3f} G（想定 1 G）",
                  otherwise='WARN')
    gyro = mean(stats.gyro_abs)
    runner.expect(gyro < 5, f"{label} の静止時の角速度", f"最大軸の平均 {gyro:.2f} deg/s（想定 0 付近）",
                  otherwise='WARN')
    if stats.quat_norms:
        quat = mean(stats.quat_norms)
        detail = f"|quat| 平均 {quat:.4f}"
        if kind == 'insole':
            runner.expect(0.95 <= quat <= 1.05, f"{label} のクォータニオン", detail + "（想定 1）")
        else:
            runner.add('INFO', f"{label} のクォータニオン", detail + "（CORE は公開版どおり Q15 で換算）")
    if kind == 'insole' and expected_header in (55, 56):
        runner.expect(stats.pressure_samples == stats.samples, f"{label} の圧力",
                      f"{stats.pressure_samples}/{stats.samples} samples に 6ch")


async def check_fifo(runner: CheckRunner, device: OrpheCoreInsole, duration: float) -> None:
    """FIFO で duration 秒収録し、欠損なく回収できるかを見る"""
    firmware = device.firmware
    supported = supports_fifo(device.kind, firmware)
    if supported is None:
        runner.add('INFO', 'FIFO', "FW 情報が読めないので確認しません（fifo コマンドで試せます）")
        return
    if not supported:
        runner.add('INFO', 'FIFO', f"FW {firmware.release_date} は非対応（{fifo_requirement(device.kind)} 以降で使えます）")
        return
    fifo = FifoRecorder(device)
    errors: list[BaseException] = []
    fifo.on('error', errors.append)
    if not runner.expect(await fifo.start(), 'FIFO の開始', str(errors[-1]) if errors else ''):
        return
    await wait_while_connected(device, duration, interval=min(duration, 0.5))
    await fifo.stop()
    runner.expect(fifo.collected_count > 0 and fifo.dropped_count == 0, 'FIFO の回収',
                  f"{fifo.collected_count} packets、欠損 {fifo.dropped_count}")
    rate = fifo_sample_rate(fifo)
    runner.expect(0.8 <= rate / FIFO_IMU_ODR_HZ <= 1.2, 'FIFO のサンプリングレート',
                  f"{rate:.1f} Hz（想定 {FIFO_IMU_ODR_HZ} Hz）", otherwise='WARN')
    samples = fifo.samples()
    if samples:
        acc = mean([math.sqrt(sum(v * v for v in sample.acc)) for sample in samples])
        runner.expect(0.9 <= acc <= 1.1, 'FIFO の静止時の加速度', f"|acc| 平均 {acc:.3f} G（想定 1 G）",
                      otherwise='WARN')
        if device.kind == 'insole':
            runner.expect(all(sample.pressure is not None for sample in samples), 'FIFO の圧力',
                          f"{len(samples)} samples に 6ch")


async def cmd_check(args: argparse.Namespace) -> int:
    runner = CheckRunner()
    log("デバイスを平らな場所に置き、終わるまで動かさないでください")
    device = await connect(args)
    if not runner.expect(device is not None, '接続'):
        return 1
    assert device is not None
    disconnected = asyncio.Event()
    device.on('disconnect', disconnected.set)
    try:
        runner.add('INFO', '種別', f"{device.kind}（{device.name}）")
        if device.time_sync is not None:
            runner.add('PASS', '時刻合わせ',
                       f"往復 {device.time_sync.average_round_trip_time:.0f} ms（{device.time_sync.round_trip_times}）")
        firmware = await device.read_firmware_info()
        runner.add('INFO', 'ファームウェア', firmware.name if firmware else '読めませんでした')
        di = await device.read_device_information()
        runner.add('PASS', 'device information の読み出し', f"battery={di.battery}")
        if device.kind == 'insole':
            calibrations = await device.read_pressure_calibrations()
            runner.add('INFO', '圧力校正係数', ' / '.join(describe_calibrations(calibrations)))

        await check_settings(runner, device, args.write)

        if device.kind == 'insole':
            for mode, header in INSOLE_MODE_HEADERS.items():
                await device.set_data_streaming_mode(mode)
                stats = await measure(device, args.duration)
                check_stream(runner, 'insole', f"mode {mode}", stats, header)
            await device.set_data_streaming_mode(DEFAULT_INSOLE_MODE)
            runner.add('INFO', 'streaming mode', f"{DEFAULT_INSOLE_MODE} に戻しました")
        else:
            stats = await measure(device, args.duration)
            check_stream(runner, 'core', 'SENSOR_VALUES', stats, None)

        await check_fifo(runner, device, args.duration)

        # 通知を止めたあとに届かないこと
        after_stop = StreamStats()
        off = device.on('sensor_values', after_stop.add_packet)
        await asyncio.sleep(1.0)
        off()
        runner.expect(after_stop.packets == 0, '通知の停止', f"停止後 1 秒間に {after_stop.packets} packets")

        if device.kind == 'core':
            await device.start_step_analysis_notification()
            await device.stop_step_analysis_notification()
        else:
            gait = InsoleGait(device)
            errors: list[BaseException] = []
            gait.on('error', errors.append)
            if not await gait.start():
                raise RuntimeError(f"歩行解析を開始できませんでした（{', '.join(map(str, errors))}）")
            await gait.stop()
        runner.add('PASS', '歩行解析の購読', "開始・停止できました。値は step で歩いて確認してください")

        await device.disconnect()
        try:
            await asyncio.wait_for(disconnected.wait(), timeout=3)
            runner.add('PASS', '切断', "disconnect イベントを受け取りました")
        except asyncio.TimeoutError:
            runner.add('FAIL', '切断', "3 秒以内に disconnect イベントが来ませんでした")
    except Exception as error:  # noqa: BLE001 (途中で失敗しても結果の要約を出す)
        runner.add('FAIL', '実行中のエラー', f"{type(error).__name__}: {error}")
    finally:
        await close(device)

    log(f"\nPASS {runner.count('PASS')} / WARN {runner.count('WARN')} / FAIL {runner.count('FAIL')}")
    return 1 if runner.count('FAIL') else 0


# ── 引数 ──────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--device', choices=('auto', 'core', 'insole'), default='auto',
                        help="探すデバイス（既定: auto = 名前で判別）")
    common.add_argument('--address', help="接続するデバイスのアドレス（scan で確認）")
    common.add_argument('--timeout', type=float, default=5.0, help="スキャン時間[s]（既定: 5）")

    parser = argparse.ArgumentParser(prog='orphe-core-insole', description="ORPHE CORE / ORPHE INSOLE を操作する")
    parser.add_argument('--version', action='version', version=f'%(prog)s {__version__}')
    sub = parser.add_subparsers(dest='command', required=True)

    scan = sub.add_parser('scan', parents=[common], help="周りのデバイスを探す")
    scan.add_argument('--all', action='store_true', help="ORPHE 以外のデバイスも表示する")
    scan.set_defaults(handler=cmd_scan)

    info = sub.add_parser('info', parents=[common], help="device information を表示する")
    info.set_defaults(handler=cmd_info)

    set_ = sub.add_parser('set', parents=[common], help="設定を書き込む")
    set_.add_argument('--acc', type=int, choices=ACC_RANGES, help="加速度レンジ[G]")
    set_.add_argument('--gyro', type=int, choices=GYRO_RANGES, help="角速度レンジ[deg/s]")
    set_.add_argument('--lr', type=int, choices=(0, 1), help="左右（CORE）")
    set_.add_argument('--led-brightness', type=int, metavar='0-255', help="LED の明るさ（CORE）")
    set_.add_argument('--led', type=int, nargs=2, metavar=('ON', 'PATTERN'), help="LED 点灯 0/1 とパターン 0-4（CORE）")
    set_.add_argument('--mount', type=int, choices=range(4), help="マウント位置（INSOLE）")
    set_.add_argument('--mode', type=int, choices=(1, 3, 4), help="streaming mode（INSOLE）")
    set_.set_defaults(handler=cmd_set)

    stream = sub.add_parser('stream', parents=[common], help="センサー値を受信する")
    stream.add_argument('--mode', type=int, choices=(1, 3, 4),
                        help="streaming mode（INSOLE。既定: 4）1=quat+IMU 200Hz, 3=IMU+圧力 200Hz, 4=全部 100Hz")
    stream.add_argument('--acc', type=int, choices=ACC_RANGES, help="受信前に加速度レンジ[G]を設定する")
    stream.add_argument('--gyro', type=int, choices=GYRO_RANGES, help="受信前に角速度レンジ[deg/s]を設定する")
    stream.add_argument('--duration', type=float, help="受信する秒数（既定: Ctrl+C まで）")
    stream.add_argument('--csv', help="サンプルを CSV に書き出す")
    stream.add_argument('--print', action='store_true', help="サンプルを標準出力に表示する")
    stream.set_defaults(handler=cmd_stream)

    fifo = sub.add_parser('fifo', parents=[common], help="FIFO（ところてん）で欠損なく収録する")
    fifo.add_argument('--duration', type=float, help="収録する秒数（既定: Ctrl+C まで）")
    fifo.add_argument('--csv', help="収録したデータを CSV に書き出す（終了時）")
    fifo.add_argument('--print', action='store_true', help="回収したサンプルを標準出力に表示する")
    fifo.add_argument('--drain-timeout', type=float, default=3.0,
                      help="停止後に取りこぼしを回収する時間[s]（無音の許容時間。既定: 3）")
    fifo.add_argument('--stop-on-loss', action='store_true', help="回復できない欠損が出たら止める")
    fifo.add_argument('--debug', action='store_true', help="MTU と、FW へのコマンド・応答をすべて表示する")
    fifo.set_defaults(handler=cmd_fifo)

    step = sub.add_parser('step', parents=[common], help="歩行解析を表示する")
    step.add_argument('--duration', type=float, help="受信する秒数（既定: Ctrl+C まで）")
    step.add_argument('--quat-distance', action='store_true',
                      help="quat_distance（INSOLE は motion。歩いていない間も届く）も表示する")
    step.add_argument('--csv', help="1 歩ずつの歩行パラメーターを CSV に書き出す（INSOLE。終了時）")
    step.set_defaults(handler=cmd_step)

    check = sub.add_parser('check', parents=[common], help="実機で主な機能を確かめる")
    check.add_argument('--duration', type=float, default=5.0, help="受信を測る秒数（モードごと。既定: 5）")
    check.add_argument('--write', action='store_true',
                       help="設定（レンジ・左右・LED・マウント位置）を別の値に書き込んで読み直し、元に戻す")
    check.set_defaults(handler=cmd_check)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return asyncio.run(args.handler(args))
    except KeyboardInterrupt:
        log("中断しました")
        return 130


if __name__ == '__main__':
    sys.exit(main())
