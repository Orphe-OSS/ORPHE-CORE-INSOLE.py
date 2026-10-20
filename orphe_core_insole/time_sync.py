"""
デバイス時計の同期（DATE_TIME キャラクタリスティック。CORE / INSOLE 共通）

ワイヤ形式は [YY(西暦-2000), MM(1-12), DD, hh, mm, ss, subsec(10ms単位)] の 7 バイト。
同期は「PC 時刻 + 平均往復時間/2」を書き込む（読み出しを数回して往復時間を測る）。
"""

from __future__ import annotations

import math
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from datetime import datetime, timedelta

DATE_TIME_LENGTH = 7
# 往復時間を測る回数の既定値
DEFAULT_TIME_SYNC_SAMPLES = 3


def encode_date_time(date: datetime) -> bytes:
    """datetime → 7 バイトのワイヤ形式"""
    return bytes([
        date.year - 2000,
        date.month,
        date.day,
        date.hour,
        date.minute,
        date.second,
        date.microsecond // 10000,
    ])


def decode_date_time(data: bytes | bytearray) -> datetime | None:
    """7 バイトのワイヤ形式 → datetime。日時として成り立たない値（時計が未設定など）は None"""
    if len(data) < DATE_TIME_LENGTH:
        return None
    try:
        # デバイスの時計は PC のローカル時刻に合わせるので、タイムゾーンは持たない
        return datetime(data[0] + 2000, data[1], data[2], data[3], data[4], data[5], data[6] * 10000)  # noqa: DTZ001
    except ValueError:
        return None


@dataclass(frozen=True)
class DeviceDateTime:
    """read_date_time() が返すデバイス時刻"""

    date: datetime | None    # デコードしたデバイス時刻（成り立たない値なら None）
    raw: bytes               # 読んだ生の値（7 バイト）
    round_trip_time: int     # 読み出しにかかった往復時間[ms]（切り捨て）


@dataclass(frozen=True)
class SyncTimeResult:
    """sync_device_time() の結果。同期のずれを後から確かめられるよう計測値も返す"""

    sum_round_trip_time: int          # 往復時間の合計[ms]
    average_round_trip_time: float    # 往復時間の平均[ms]
    standard_time: int                # 書き込み直前の PC 時刻（epoch ms）
    adjusted_time: int                # デバイスへ書いた時刻（standard_time + 片道遅延、epoch ms）
    round_trip_times: list[int]       # 各回の往復時間[ms]
    half_round_trip_time: int         # 片道遅延の推定値[ms]（平均往復時間の半分）


async def read_date_time(read: Callable[[], Awaitable[bytes | bytearray]],
                         clock: Callable[[], float] = time.perf_counter) -> DeviceDateTime:
    """デバイスの時刻を読み、往復時間を添えて返す。clock は単調クロック[s]"""
    start = clock()
    data = bytes(await read())
    round_trip_time = math.floor((clock() - start) * 1000)
    return DeviceDateTime(decode_date_time(data), data, round_trip_time)


async def sync_device_time(read: Callable[[], Awaitable[bytes | bytearray]],
                           write: Callable[[bytes], Awaitable[None]],
                           samples: int = DEFAULT_TIME_SYNC_SAMPLES,
                           now: Callable[[], datetime] = datetime.now,
                           clock: Callable[[], float] = time.perf_counter) -> SyncTimeResult:
    """デバイスの時計を PC 時刻 + 平均往復時間/2 に合わせる"""
    if samples < 1:
        raise ValueError("samples must be 1 or more")
    round_trip_times = [(await read_date_time(read, clock)).round_trip_time for _ in range(samples)]
    sum_round_trip_time = sum(round_trip_times)
    average_round_trip_time = sum_round_trip_time / samples
    half_round_trip_time = math.floor(average_round_trip_time / 2 + 0.5)
    standard = now()
    adjusted = standard + timedelta(milliseconds=half_round_trip_time)

    await write(encode_date_time(adjusted))
    return SyncTimeResult(
        sum_round_trip_time=sum_round_trip_time,
        average_round_trip_time=average_round_trip_time,
        standard_time=_epoch_ms(standard),
        adjusted_time=_epoch_ms(adjusted),
        round_trip_times=round_trip_times,
        half_round_trip_time=half_round_trip_time,
    )


def _epoch_ms(date: datetime) -> int:
    return math.floor(date.timestamp() * 1000)
