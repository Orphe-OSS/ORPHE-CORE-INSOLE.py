"""FIFO（ところてん）の FW を模擬する BleakClient の代わり"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from typing import Any

from helpers import SimulatedClient, firmware_name

from orphe_core_insole.uuids import CHARACTERISTIC_FIRMWARE_NAME_UUID, CHARACTERISTIC_SENSOR_VALUES_UUID

# (デバイス種別, ヘッダ) → (サンプル数, 1 サンプルのバイト数)
LAYOUTS = {('insole', 0x36): (4, 24), ('core', 0x36): (8, 12)}


def expected_raw(serial: int, index: int, acc_z: int = 2048
                 ) -> tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, ...]]:
    """シリアル番号とパケット内の位置（0 が最新）から決まるサンプルの生値。acc_z の既定は ±16G での 1G"""
    gyro = (serial % 1000, -(serial % 1000), index)
    acc = (0, 0, acc_z)
    pressure = tuple((serial + j) % 4096 for j in range(6))
    return gyro, acc, pressure


def build_fifo_packet(kind: str, header: int, serial: int, now_ms: int, acc_z: int = 2048) -> bytes:
    samples, size = LAYOUTS[(kind, header)]
    ms_of_day = now_ms % 86_400_000
    h, rest = divmod(ms_of_day, 3_600_000)
    m, rest = divmod(rest, 60_000)
    s, ms = divmod(rest, 1000)
    data = bytearray([header]) + serial.to_bytes(2, 'big') + bytes([h, m, s]) + ms.to_bytes(2, 'big')
    for index in range(samples):
        gyro, acc, pressure = expected_raw(serial, index, acc_z)
        data += b''.join(v.to_bytes(2, 'big', signed=True) for v in gyro + acc)
        if size == 24:
            data += b''.join(v.to_bytes(2, 'big') for v in pressure)
    return bytes(data)


class FifoClient(SimulatedClient):
    """
    FIFO に対応した FW を模擬する（リアルタイム通信は SimulatedClient のまま）

    Args:
        kind: 'core' / 'insole'
        header: データパケットのヘッダ
        packets_per_sec: FW がパケットを作る速さ（既定は 208Hz ÷ 1 パケットのサンプル数）
        drop: (serial, 何回目の送信か) → True ならその notify を落とす（BLE の取りこぼし）
        ring_capacity: FW が保持するパケット数
        release_date: GET_FW_NAME のリリース日（None なら読めない）
        ack_drop: 落とす ACK の sub と回数
        max_write: 1 回で書ける長さ。超えた書き込みは届かない（macOS の応答なし書き込みと同じ）
    """

    def __init__(self, device: Any = None, *, kind: str = 'insole', header: int = 0x36,
                 packets_per_sec: float | None = None, drop: Callable[[int, int], bool] | None = None,
                 ring_capacity: int = 1500, release_date: int | None = 20260901,
                 ack_drop: dict[int, int] | None = None, max_write: int | None = None,
                 calibrations: list[tuple[int, tuple[float, ...]] | None] | None = None) -> None:
        # 対応 FW は校正係数の要求に応答する（既定は全 ch 未書き込み）
        super().__init__(device, kind, calibrations=calibrations if calibrations is not None else [None] * 6)
        self.header = header
        self.packets_per_sec = packets_per_sec or 208 / LAYOUTS[(kind, header)][0]
        self.drop = drop or (lambda serial, attempt: False)
        self.ring_capacity = ring_capacity
        self.release_date = release_date
        self.ack_drop = dict(ack_drop or {})
        self.mode_writes: list[int] = []
        self.monitoring = False
        self.ring: dict[int, bytes] = {}
        self.next_serial = 1000
        self.generated = 0
        self.started_at = 0.0
        self.attempts: dict[int, int] = {}
        self.commands: list[int] = []
        self.max_write = max_write
        # 受信バッファ。短い書き込みのときは後ろに前回の内容が残るとして扱い、終端の count=0 で止まるかを確かめる
        self.di_buffer = bytearray(122)
        if max_write is not None:
            self.services = _Services(max_write)

    async def read_gatt_char(self, uuid: str) -> bytearray:
        if uuid == CHARACTERISTIC_FIRMWARE_NAME_UUID:
            if self.release_date is None:
                raise RuntimeError("characteristic not found")
            return bytearray(firmware_name(self.release_date))
        return await super().read_gatt_char(uuid)

    def _now(self) -> float:
        return asyncio.get_running_loop().time()

    def _generate(self) -> None:
        """蓄積中なら、経過時間ぶんのパケットをリングバッファに足す"""
        if not self.monitoring:
            return
        target = int((self._now() - self.started_at) * self.packets_per_sec)
        while self.generated < target:
            serial = self.next_serial
            now_ms = 12 * 3_600_000 + int(self.generated * 1000 / self.packets_per_sec)
            acc_z = round(32768 / [2, 4, 8, 16][self.state[8]])  # device information のレンジで 1G
            self.ring[serial] = build_fifo_packet(self.kind, self.header, serial, now_ms, acc_z)
            self.next_serial = (serial + 1) % 65536
            self.generated += 1
            while len(self.ring) > self.ring_capacity:
                del self.ring[next(iter(self.ring))]

    def _notify(self, packets: list[bytes]) -> None:
        callback = self.notify.get(CHARACTERISTIC_SENSOR_VALUES_UUID)
        if callback is None:
            return

        async def send() -> None:
            for packet in packets:
                await asyncio.sleep(0)
                if self.is_connected:
                    await callback(None, bytearray(packet))

        asyncio.get_running_loop().create_task(send())

    def _ack(self, sub: int) -> None:
        if self.ack_drop.get(sub, 0) > 0:
            self.ack_drop[sub] -= 1
            return
        self._notify([bytes([0x35, sub])])

    async def write_gatt_char(self, uuid: str, data: bytes | bytearray) -> None:
        if self.max_write is not None and len(data) > self.max_write:
            self.writes.append((uuid, bytes(data)))
            return  # 黙って捨てられる
        await super().write_gatt_char(uuid, data)
        self.di_buffer[:len(data)] = data
        self._generate()
        if data[0] == 0x0D:
            self.mode_writes.append(data[1])
            return
        if data[0] != 0x0B:
            return
        sub = data[1]
        self.commands.append(sub)
        if sub == 0x06:
            self.monitoring = False
            self._ack(sub)
        elif sub == 0x03:
            self.ring.clear()
            self._ack(sub)
        elif sub == 0x04:
            self.monitoring = True
            self.started_at = self._now()
            self.generated = 0
            self._ack(sub)
        elif sub == 0x01:
            last = (self.next_serial - 1) % 65536
            watermark = 1 if len(self.ring) >= self.ring_capacity else 0
            self._notify([bytes([0x35, 0x01]) + last.to_bytes(2, 'big') + bytes([watermark])
                          + len(self.ring).to_bytes(2, 'big')])
        elif sub == 0x02:
            self._notify(self._respond_data(bytes(self.di_buffer)))

    def _respond_data(self, data: bytes | bytearray) -> list[bytes]:
        packets: list[bytes] = []
        for k in range(30):
            o = 2 + 4 * k
            start = int.from_bytes(data[o:o + 2], 'big')
            count = int.from_bytes(data[o + 2:o + 4], 'big')
            if count == 0:
                break  # FW は count=0 の組で読むのをやめる
            missing_start, missing_count = None, 0
            for i in range(count):
                serial = (start + i) % 65536
                packet = self.ring.get(serial)
                if packet is None:
                    if missing_start is None:
                        missing_start = serial
                    missing_count += 1
                    continue
                if missing_start is not None:
                    packets.append(bytes([0x35, 0x02]) + missing_start.to_bytes(2, 'big')
                                   + missing_count.to_bytes(2, 'big'))
                    missing_start, missing_count = None, 0
                attempt = self.attempts.get(serial, 0) + 1
                self.attempts[serial] = attempt
                if not self.drop(serial, attempt):
                    packets.append(packet)
            if missing_start is not None:
                packets.append(bytes([0x35, 0x02]) + missing_start.to_bytes(2, 'big')
                               + missing_count.to_bytes(2, 'big'))
        return packets


class _Characteristic:
    def __init__(self, max_write: int) -> None:
        self.max_write_without_response_size = max_write


class _Services:
    def __init__(self, max_write: int) -> None:
        self._characteristic = _Characteristic(max_write)

    def get_characteristic(self, uuid: str) -> _Characteristic:
        return self._characteristic
