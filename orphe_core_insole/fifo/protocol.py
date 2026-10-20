"""
FIFO（ところてん）— ロスレス収録のプロトコル

リアルタイム通信は BLE で落ちたパケットが戻らない。FIFO は FW のリングバッファに溜まった
サンプルを「シリアル番号を指定して取り出す」プル型の通信で、落ちたぶんは再要求して回収する。

BLE の手順（コマンドは DEVICE_INFORMATION に write、応答は SENSOR_VALUES の notify）:
    [0x0D, 0x02]                        読み取りモードを FIFO にする
    [0x0B, 0x01]                        現在のシリアル番号 → 0x35 0x01 serial(2) watermark(1) accumulated(2)
    [0x0B, 0x02, 30×(start(2), count(2))]  範囲を指定してデータを要求（30 組に満たなければ 0 埋め）
    [0x0B, 0x03] / 0x04 / 0x06          全消去 / 蓄積開始 / 蓄積停止（ACK: 0x35 0x03 / 0x04 / 0x06）
    no-data 応答:  0x35 0x02 start(2) count(2)
    データパケット: header serial(2) 時 分 秒 ms(2) + サンプル × N

データパケットのレイアウト（サンプルはパケットの先頭ほど新しい）:
    INSOLE 0x36: 4 サンプル × 24B（gyro 3, acc 3, 圧力 6）= 104B
    CORE   0x36: 8 サンプル × 12B（gyro 3, acc 3）= 104B（FW 20260905 以降）

FIFO にクォータニオンは入らない。サンプリングは IMU（LSM6DSOX）の実測 ODR 約 208Hz。
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from decimal import ROUND_HALF_UP, Decimal, localcontext

from ..pressure import PressureCalibration, pressure_to_newton
from ..profiles.base import DEFAULT_ACC_RANGE, DEFAULT_GYRO_RANGE, gyro_raw_to_dps

UINT16_MAX = 65536

# 読み取りモード [0x0D, mode]
OP_READ_MODE = 0x0D
FIFO_READ_MODE = 0x02
# CORE の 0x0D は 1: リアルタイム / 2: FIFO の 2 値だけ（INSOLE の streaming mode 1/3/4 とは別物）。
# CORE に INSOLE 流の 0x04 を書くと切断されるので、CORE はこの値で復帰する
CORE_REALTIME_READ_MODE = 0x01
INSOLE_DEFAULT_STREAMING_MODE = 4

# FIFO コマンド [0x0B, sub]
OP_INFO = 0x0B
SUB_GET_SERIAL = 0x01
SUB_GET_DATA = 0x02
SUB_DELETE_ALL = 0x03
SUB_START_MONITOR = 0x04
SUB_STOP_MONITOR = 0x06

# 応答のヘッダ
RESP_STATUS = 0x35
RESP_NO_DATA_SUB = 0x02
HEADER_DATA = 0x36

# 1 回のデータ要求に載せられる (start, count) の組数（FW は固定 30 組で読む）
FIFO_RE_REQUEST_DATA_NUM = 30
# 1 回の要求で頼むシリアル数の上限
MAX_DATA_NUMBER_REQUESTED_AT_ONCE = 200
# 次のループへ持ち越す再要求の上限。超えたぶんは回復不能として計上する
FIFO_MAX_CARRY_OVER_SERIALS = 100
# FW のリングバッファの保持件数の目安。これ以上遅れると古いサンプルが上書きされる
FIFO_RING_BUFFER_CAPACITY = 1500
# stop() 後の回収フェーズで、無音をどれだけ許すか[s]。データが届く間は延長し、× 10 で打ち切る
DEFAULT_DRAIN_TIMEOUT_SEC = 3.0
FIFO_CATCHUP_MAX_BUDGET_FACTOR = 10

# IMU（LSM6DSOX）の実測 ODR[Hz] と 1 サンプルの時間[ms]
FIFO_IMU_ODR_HZ = 208
FIFO_FRAME_INTERVAL_MS = 1000 / FIFO_IMU_ODR_HZ
# CSV の timestamp 列だけは既存の記録データに揃えて 5ms 刻みにする（dt の計算には使わないこと）
FIFO_LEGACY_CSV_FRAME_INTERVAL_MS = 5

FIFO_CSV_HEADER_PRESSURE = (
    'serial_number,timestamp,'
    'gyro_x[dps],gyro_y[dps],gyro_z[dps],'
    'acc_x[G],acc_y[G],acc_z[G],'
    'press1[N],press2[N],press3[N],press4[N],press5[N],press6[N]'
)
FIFO_CSV_HEADER_IMU = (
    'serial_number,timestamp,'
    'gyro_x[dps],gyro_y[dps],gyro_z[dps],'
    'acc_x[G],acc_y[G],acc_z[G]'
)


# ── パケットのレイアウト ─────────────────────────────────────────

@dataclass(frozen=True)
class FifoLayout:
    """FIFO データパケットのレイアウト"""

    header: int
    samples: int        # 1 パケットのサンプル数
    sample_size: int    # 1 サンプルのバイト数
    has_pressure: bool

    @property
    def packet_size(self) -> int:
        return 8 + self.samples * self.sample_size


INSOLE_LAYOUTS = {
    HEADER_DATA: FifoLayout(HEADER_DATA, 4, 24, True),
}
CORE_LAYOUTS = {
    HEADER_DATA: FifoLayout(HEADER_DATA, 8, 12, False),
}


def layouts_for(kind: str) -> Mapping[int, FifoLayout]:
    return CORE_LAYOUTS if kind == 'core' else INSOLE_LAYOUTS


def layout_of(data: bytes | bytearray, kind: str) -> FifoLayout | None:
    """データパケットならレイアウト、そうでなければ None（長さも確かめる）"""
    if len(data) < 1:
        return None
    layout = layouts_for(kind).get(data[0])
    if layout is None or len(data) != layout.packet_size:
        return None
    return layout


# ── 単位換算 ─────────────────────────────────────────────────────

def i16(data: bytes | bytearray, at: int) -> int:
    return int.from_bytes(data[at:at + 2], byteorder='big', signed=True)


def u16(data: bytes | bytearray, at: int) -> int:
    return int.from_bytes(data[at:at + 2], byteorder='big', signed=False)


def polynomial_pressure_to_newton(x: float, n: int) -> float:
    """圧力の ADC 値を N に換算する（校正係数が無いときの ch ごとの固定多項式。n は 1..6、負値は 0）"""
    x = float(x)  # 浮動小数点で累乗する
    if n == 1:
        y = 6.31278e-11 * x ** 4 - 2.33093e-07 * x ** 3 + 3.27825e-04 * x ** 2 - 1.63373e-01 * x + 2.25012e01
    elif n == 2:
        y = 6.65168e-11 * x ** 4 - 2.10741e-07 * x ** 3 + 2.31937e-04 * x ** 2 - 7.10366e-02 * x + 6.97927e00
    elif n == 3:
        y = 1.07646e-10 * x ** 4 - 3.85112e-07 * x ** 3 + 5.02384e-04 * x ** 2 - 2.37328e-01 * x + 3.18015e01
    elif n == 4:
        y = 5.91156e-11 * x ** 4 - 1.81045e-07 * x ** 3 + 1.86644e-04 * x ** 2 - 4.46178e-02 * x + 3.10811e00
    elif n == 5:
        y = 5.32573e-11 * x ** 4 - 1.68515e-07 * x ** 3 + 1.79518e-04 * x ** 2 - 4.66859e-02 * x + 3.71484e00
    elif n == 6:
        y = 4.44324e-11 * x ** 4 - 1.09728e-07 * x ** 3 + 8.90389e-05 * x ** 2 + 3.82816e-03 * x - 4.46580e00
    else:
        y = 0.0
    return 0.0 if y < 0 else y


@dataclass(frozen=True)
class FifoScale:
    """生値を物理値にする係数（device information のレンジから決める）"""

    acc_g: int = DEFAULT_ACC_RANGE      # 加速度レンジ[G]
    gyro_dps: int = DEFAULT_GYRO_RANGE  # 角速度レンジ[deg/s]

    def acc(self, raw: int) -> float:
        return raw / 32768.0 * self.acc_g

    def gyro(self, raw: int) -> float:
        return gyro_raw_to_dps(raw, self.gyro_dps)


# ── シリアル番号 ─────────────────────────────────────────────────

FifoRequestRange = tuple[int, int]  # (開始シリアル, 件数)


def serial_distance(start_exclusive: int, end_inclusive: int) -> int:
    """start_exclusive の次から end_inclusive まで進んだ個数（65535 → 0 の巻き戻り対応）"""
    return (end_inclusive - start_exclusive) % UINT16_MAX


def calc_expected_serials(start_serial: int, request_size: int) -> list[int]:
    return [(start_serial + i) % UINT16_MAX for i in range(request_size)]


def expand_requests_to_list(requests: Iterable[FifoRequestRange]) -> list[int]:
    return [(start + i) % UINT16_MAX for start, count in requests for i in range(count)]


def build_requests_from_serials(serials: Iterable[int]) -> list[FifoRequestRange]:
    """シリアル番号の集合を連番の塊にまとめる 例: {10, 12, 40, 41, 42} → [(10, 1), (12, 1), (40, 3)]"""
    ordered = sorted(serials)
    if not ordered:
        return []
    out: list[FifoRequestRange] = []
    run_start = prev = ordered[0]
    run_len = 1
    for serial in ordered[1:]:
        if serial == (prev + 1) % UINT16_MAX:
            run_len += 1
        else:
            out.append((run_start, run_len))
            run_start, run_len = serial, 1
        prev = serial
    out.append((run_start, run_len))
    return out


# ── コマンドと応答 ───────────────────────────────────────────────

FIFO_REQUEST_FULL_SIZE = 2 + 4 * FIFO_RE_REQUEST_DATA_NUM  # 122 バイト


def create_get_sensor_data_request(pairs: list[FifoRequestRange], max_size: int = FIFO_REQUEST_FULL_SIZE) -> bytes:
    """
    データ要求 [0x0B, 0x02, (start, count) × 30 組]（不足は 0 埋めで 122 バイト）

    max_size が 122 より小さいとき（1 回で書き込める長さが足りないとき）は、要求する組のあとに
    count=0 の組を 1 つだけ付けて短く送る（count=0 の組が要求の終わりを表す）。
    """
    if len(pairs) > max_pairs_per_request(max_size):
        raise ValueError(f"too many ranges ({len(pairs)} > {max_pairs_per_request(max_size)})")
    request = bytearray([OP_INFO, SUB_GET_DATA])
    for serial, count in pairs:
        request += serial.to_bytes(2, 'big') + count.to_bytes(2, 'big')
    slots = FIFO_RE_REQUEST_DATA_NUM if max_size >= FIFO_REQUEST_FULL_SIZE else len(pairs) + 1
    request += bytes(4 * (slots - len(pairs)))
    return bytes(request)


def max_pairs_per_request(max_size: int) -> int:
    """1 回の要求に載せられる組数（短く送るときは終端の 0 の組のぶん 1 つ減る）"""
    if max_size >= FIFO_REQUEST_FULL_SIZE:
        return FIFO_RE_REQUEST_DATA_NUM
    return max(1, (max_size - 2) // 4 - 1)


def parse_no_data_response(data: bytes | bytearray) -> tuple[int, int] | None:
    """no-data 応答（0x35 0x02 start count）→ (start, count)。足りないフィールドは 0"""
    if len(data) < 2 or data[0] != RESP_STATUS or data[1] != RESP_NO_DATA_SUB:
        return None
    start = u16(data, 2) if len(data) >= 4 else 0
    count = u16(data, 4) if len(data) >= 6 else 0
    return start, count


@dataclass(frozen=True)
class FifoCurrentSerial:
    """現在のシリアル番号の応答（0x35 0x01）"""

    serial: int         # FW が最後に書き込んだシリアル番号
    watermark: int      # リングバッファが満杯なら 1
    accumulated: int    # FW に溜まっている件数


def parse_current_serial(data: bytes | bytearray) -> FifoCurrentSerial | None:
    if len(data) < 7 or data[0] != RESP_STATUS or data[1] != SUB_GET_SERIAL:
        return None
    return FifoCurrentSerial(u16(data, 2), data[4], u16(data, 5))


def extract_timestamp_ms(data: bytes | bytearray) -> int:
    """パケットの時刻（その日の 0 時からの経過 ms）"""
    return (data[3] * 3600 + data[4] * 60 + data[5]) * 1000 + u16(data, 6)


def timestamp_to_str(hour: int, minute: int, second: int, ms: int, offset_ms: int) -> str:
    """HH:MM:SS:mmm（オフセットを足して繰り上げる）"""
    ms += offset_ms
    while ms >= 1000:
        ms -= 1000
        second += 1
    while second >= 60:
        second -= 60
        minute += 1
    while minute >= 60:
        minute -= 60
        hour += 1
    while hour >= 24:
        hour -= 24
    return f"{hour:02d}:{minute:02d}:{second:02d}:{ms:03d}"


# ── デコード ─────────────────────────────────────────────────────

@dataclass(frozen=True)
class FifoSample:
    """
    FIFO の 1 サンプル

    Attributes:
        serial_number: パケットのシリアル番号
        packet_number: パケット内の位置（0 が最も古い）
        t: パケットの時刻 + packet_number × 約 4.81ms（その日の 0 時からの ms）。dt はこれで計算する
        gyro: 角速度 (x, y, z)[deg/s]
        acc: 加速度 (x, y, z)[G]
        pressure: 6ch の圧力（ADC 値）。CORE は None
    """

    serial_number: int
    packet_number: int
    t: float
    gyro: tuple[float, float, float]
    acc: tuple[float, float, float]
    pressure: tuple[int, ...] | None


@dataclass(frozen=True)
class _RawFrame:
    gyro: tuple[int, int, int]
    acc: tuple[int, int, int]
    pressure: tuple[int, ...] | None


def _read_frames(data: bytes | bytearray, layout: FifoLayout) -> list[_RawFrame]:
    """古い順（バイト列の末尾のサンプルから）に並べる"""
    frames = []
    for i in range(layout.samples - 1, -1, -1):
        o = 8 + i * layout.sample_size
        pressure = tuple(u16(data, o + 12 + 2 * j) for j in range(6)) if layout.has_pressure else None
        frames.append(_RawFrame(
            (i16(data, o), i16(data, o + 2), i16(data, o + 4)),
            (i16(data, o + 6), i16(data, o + 8), i16(data, o + 10)),
            pressure,
        ))
    return frames


def decode_fifo_packet(data: bytes | bytearray, layout: FifoLayout, scale: FifoScale) -> list[FifoSample]:
    serial = u16(data, 1)
    base_ms = extract_timestamp_ms(data)
    samples = []
    for packet_number, frame in enumerate(_read_frames(data, layout)):
        samples.append(FifoSample(
            serial, packet_number, base_ms + packet_number * FIFO_FRAME_INTERVAL_MS,
            (scale.gyro(frame.gyro[0]), scale.gyro(frame.gyro[1]), scale.gyro(frame.gyro[2])),
            (scale.acc(frame.acc[0]), scale.acc(frame.acc[1]), scale.acc(frame.acc[2])),
            frame.pressure,
        ))
    return samples


# ── CSV ──────────────────────────────────────────────────────────

def _to_fixed(value: float, digits: int) -> str:
    """小数 digits 桁の文字列にする（0.5 は絶対値の大きい方へ。-0 は 0。1e21 以上は指数表記）"""
    if abs(value) >= 1e21:
        return repr(value)
    with localcontext() as context:
        context.prec = 64
        text = str(Decimal(value).quantize(Decimal(1).scaleb(-digits), rounding=ROUND_HALF_UP))
    return text[1:] if value == 0 and text.startswith('-') else text


def packet_to_csv_rows(data: bytes | bytearray, layout: FifoLayout, scale: FifoScale,
                       calibrations: Sequence[PressureCalibration | None] | None = None) -> list[str]:
    """
    1 パケット → CSV の行（古い順）。timestamp 列は 5ms 刻み

    圧力は calibrations（read_pressure_calibrations() の結果）があれば ch ごとの係数で N にする
    （係数が無い ch は固定式）。calibrations が無ければ ch ごとの固定多項式で N にする。
    """
    serial = u16(data, 1)
    h, m, s, ms = data[3], data[4], data[5], u16(data, 6)
    rows = []
    for packet_number, frame in enumerate(_read_frames(data, layout)):
        columns = [
            str(serial), timestamp_to_str(h, m, s, ms, packet_number * FIFO_LEGACY_CSV_FRAME_INTERVAL_MS),
            *(_to_fixed(scale.gyro(v), 2).rjust(8) for v in frame.gyro),
            *(_to_fixed(scale.acc(v), 4).rjust(8) for v in frame.acc),
        ]
        if frame.pressure is not None:
            columns += [_to_fixed(_csv_pressure(v, j, calibrations), 4).rjust(8) for j, v in enumerate(frame.pressure)]
        rows.append(', '.join(columns))
    return rows


def _csv_pressure(raw: int, index: int, calibrations: Sequence[PressureCalibration | None] | None) -> float:
    if calibrations is not None:
        return pressure_to_newton(raw, calibrations[index])
    return polynomial_pressure_to_newton(raw, index + 1)


def raw_store_to_csv(raw_store: Mapping[int, bytes], kind: str, scale: FifoScale,
                     calibrations: Sequence[PressureCalibration | None] | None = None) -> str:
    """回収したパケットを時刻順に並べて CSV 文字列にする"""
    has_pressure = kind != 'core'
    lines = [FIFO_CSV_HEADER_PRESSURE if has_pressure else FIFO_CSV_HEADER_IMU]
    packets = [(data, layout) for data in raw_store.values() if (layout := layout_of(data, kind)) is not None]
    for data, layout in sorted(packets, key=lambda item: extract_timestamp_ms(item[0])):
        lines += packet_to_csv_rows(data, layout, scale, calibrations)
    return '\n'.join(lines) + '\n'
