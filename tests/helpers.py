"""テスト用のパケット生成と、bleak の代わりに使う偽物"""

from __future__ import annotations

import asyncio
import struct
from collections.abc import Callable
from typing import Any

from orphe_core_insole.uuids import (
    CHARACTERISTIC_DATE_TIME_UUID,
    CHARACTERISTIC_DEVICE_INFORMATION_UUID,
    CHARACTERISTIC_FIRMWARE_NAME_UUID,
    CHARACTERISTIC_SENSOR_VALUES_UUID,
    CHARACTERISTIC_STEP_ANALYSIS_UUID,
)


def i16(value: int) -> bytes:
    return int(value).to_bytes(2, byteorder="big", signed=True)


def u16(value: int) -> bytes:
    return int(value).to_bytes(2, byteorder="big", signed=False)


PACKET_SIZE = 104
# data[3..7]: 時, 分, 秒, ミリ秒上位, ミリ秒下位
HEADER_TIME = bytes([12, 30, 15, 0, 100])


def pad(payload: bytes, size: int = PACKET_SIZE) -> bytes:
    if len(payload) > size:
        raise ValueError(f"packet too long: {len(payload)}")
    return payload + bytes(size - len(payload))


def build_packet_50(quat, gyro, acc, frames=4, serial=1, elapsed_ms=5) -> bytes:
    """ヘッダ50（quat+gyro+acc, 21バイト間隔）。CORE と INSOLE mode 1 で共通のレイアウト"""
    payload = bytes([50]) + u16(serial) + HEADER_TIME
    for _ in range(frames):
        payload += b"".join(i16(v) for v in quat)
        payload += b"".join(i16(v) for v in gyro)
        payload += b"".join(i16(v) for v in acc)
        payload += bytes([elapsed_ms])
    return pad(payload)


def build_packet_55(gyro, acc, press, frames=4, serial=2) -> bytes:
    """ヘッダ55（INSOLE mode 3: gyro+acc+press, 24バイト間隔）"""
    payload = bytes([55]) + u16(serial) + HEADER_TIME
    for _ in range(frames):
        payload += b"".join(i16(v) for v in gyro)
        payload += b"".join(i16(v) for v in acc)
        payload += b"".join(u16(v) for v in press)
    return pad(payload)


def build_packet_56(quat, gyro, acc, press, frames=2, serial=3) -> bytes:
    """ヘッダ56（INSOLE mode 4: quat+gyro+acc+press, 32バイト間隔）"""
    payload = bytes([56]) + u16(serial) + HEADER_TIME
    for _ in range(frames):
        payload += b"".join(i16(v) for v in quat)
        payload += b"".join(i16(v) for v in gyro)
        payload += b"".join(i16(v) for v in acc)
        payload += b"".join(u16(v) for v in press)
    return pad(payload)


def build_packet_40(quat, gyro, acc, timestamp=1234) -> bytes:
    """ヘッダ40（CORE 50Hz モデル: quat は int16、gyro / acc は int8）"""
    payload = bytes([40]) + b"".join(i16(v) for v in quat)
    payload += bytes(v & 0xFF for v in gyro) + bytes(2)
    payload += bytes(v & 0xFF for v in acc) + bytes(1)
    payload += u16(timestamp)
    return pad(payload, 20)


def build_step_analysis(sub_header: int, step_count: int, flags: int | None = None,
                        floats: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)) -> bytes:
    """STEP_ANALYSIS パケット（4-19 は float32 × 4。flags を渡すと data[4] を上書きする）"""
    body = b"".join(struct.pack('>f', v) for v in floats)
    data = bytearray(bytes([0, sub_header]) + u16(step_count) + body)
    if flags is not None:
        data[4] = flags
    return bytes(data)


# CORE の device information: battery, lr, rec, auto_run, led, -, log_high, log_low, acc, gyro
CORE_DEVICE_INFORMATION = bytes([80, 1, 0, 1, 200, 0, 3, 4, 2, 3]) + bytes(10)
# INSOLE の device information: battery, mount, ..., acc(8), gyro(9), ..., version(18)
INSOLE_DEVICE_INFORMATION = bytes([70, 1, 0, 0, 0, 0, 0, 0, 3, 3]) + bytes(8) + bytes([5, 0])


class FakeClient:
    """BleakClient の代わり。書き込みと notify の購読を記録する"""

    def __init__(self, device: Any = None, device_information: bytes = CORE_DEVICE_INFORMATION) -> None:
        self.device = device
        self.is_connected = False
        self.device_information = device_information
        self.writes: list[tuple[str, bytes]] = []
        self.notify: dict[str, Callable[..., Any]] = {}
        self.mtu_size = 247
        # デバイスの時計（DATE_TIME の値）
        self.date_time = bytearray([26, 1, 1, 0, 0, 0, 0])

    async def connect(self) -> bool:
        self.is_connected = True
        return True

    async def disconnect(self) -> bool:
        self.is_connected = False
        return True

    async def read_gatt_char(self, uuid: str) -> bytearray:
        if uuid == CHARACTERISTIC_DATE_TIME_UUID:
            return bytearray(self.date_time)
        assert uuid == CHARACTERISTIC_DEVICE_INFORMATION_UUID
        return bytearray(self.device_information)

    async def write_gatt_char(self, uuid: str, data: bytes | bytearray) -> None:
        self.writes.append((uuid, bytes(data)))
        if uuid == CHARACTERISTIC_DATE_TIME_UUID:
            self.date_time = bytearray(data)

    async def start_notify(self, uuid: str, callback: Callable[..., Any]) -> None:
        self.notify[uuid] = callback

    async def stop_notify(self, uuid: str) -> None:
        self.notify.pop(uuid, None)


class FakeBLEDevice:
    def __init__(self, name: str | None, address: str) -> None:
        self.name = name
        self.address = address


class FakeAdvertisement:
    def __init__(self, rssi: int) -> None:
        self.rssi = rssi


def fake_discover(*devices: tuple[str | None, str]):
    """BleakScanner.discover(return_adv=True) の代わり"""

    async def discover(timeout: float) -> dict[str, tuple[FakeBLEDevice, FakeAdvertisement]]:
        return {address: (FakeBLEDevice(name, address), FakeAdvertisement(-50))
                for name, address in devices}

    return discover


class Recorder:
    """on() で受け取ったイベントを順に記録する"""

    def __init__(self) -> None:
        self.events: list[tuple[str, tuple[Any, ...]]] = []

    def listen(self, device: Any, *events: str) -> None:
        for event in events:
            device.on(event, lambda *args, e=event: self.events.append((e, args)))

    def names(self) -> list[str]:
        return [name for name, _ in self.events]

    def of(self, event: str) -> list[Any]:
        return [args[0] if len(args) == 1 else args for name, args in self.events if name == event]


def firmware_name(release_date: int, build_id: str = "INS-TEST") -> bytes:
    """GET_FW_NAME の値: build_id(15) major(1) minor(2) year(2) month day patch"""
    year, month, day = release_date // 10000, release_date // 100 % 100, release_date % 100
    return (build_id.encode().ljust(15, b'\0') + bytes([1]) + (3).to_bytes(2, 'big')
            + year.to_bytes(2, 'big') + bytes([month, day, 7]))


def gait_step_packets(step: int) -> list[bytes]:
    """INSOLE の歩行解析 1 歩ぶん（overview / stride / pronation。それぞれ 20 バイト）"""
    def f32(*values: float) -> bytes:
        return struct.pack(f'>{len(values)}f', *values)

    head = u16(step)
    return [
        bytes([51, 0]) + head + bytes([0b01001000, 0]) + u16(0x3E00) + f32(1.2, 0.5, 0.25),
        bytes([51, 1]) + head + f32(5.5, 0.9, 0.1, 0.0),
        bytes([51, 2]) + head + f32(1.25, -5.0, -9.4, 2.0),
    ]


def calibration_packet(sensor_index: int, func: int, coefficients: tuple[float, ...]) -> bytes:
    """校正係数の notify（0x39 ch func double×5）"""
    return bytes([0x39, sensor_index, func]) + struct.pack('>5d', *coefficients)


class SimulatedClient(FakeClient):
    """
    実機のように振る舞う BleakClient の代わり

    device information への書き込みを反映し、notify を購読すると静止状態のパケットを
    実機と同じレート（1 パケット 20ms）で送り続ける。
    """

    PACKET_INTERVAL_SEC = 0.02

    def __init__(self, device: Any, kind: str, apply_writes: bool = True, serial_start: int = 0,
                 release_date: int | None = None,
                 calibrations: list[tuple[int, tuple[float, ...]] | None] | None = None,
                 steps: int = 0) -> None:
        base = CORE_DEVICE_INFORMATION if kind == 'core' else INSOLE_DEVICE_INFORMATION
        super().__init__(device, base)
        self.kind = kind
        self.apply_writes = apply_writes
        self.state = bytearray(base)
        self.mode = 4
        # 最後に書かれた読み取りモード（2 = FIFO。FIFO 中はリアルタイムのパケットを送らない）
        self.read_mode: int | None = None
        self.serial = serial_start
        self.tasks: dict[str, Any] = {}
        # GET_FW_NAME のリリース日（None なら読めない FW）
        self.release_date = release_date
        # INSOLE の校正係数（ch ごとの (func, 係数)。None の ch は未書き込み）。None なら応答しない FW
        self.calibrations = calibrations
        self.calibration_requests: list[int] = []
        # STEP_ANALYSIS を購読したら送る歩数（INSOLE の歩行解析）
        self.steps = steps

    async def read_gatt_char(self, uuid: str) -> bytearray:
        if uuid == CHARACTERISTIC_FIRMWARE_NAME_UUID:
            if self.release_date is None:
                raise RuntimeError("characteristic not found")
            return bytearray(firmware_name(self.release_date))
        if uuid == CHARACTERISTIC_DATE_TIME_UUID:
            return bytearray(self.date_time)
        return bytearray(self.state)

    async def write_gatt_char(self, uuid: str, data: bytes | bytearray) -> None:
        await super().write_gatt_char(uuid, data)
        if not self.apply_writes or uuid == CHARACTERISTIC_DATE_TIME_UUID:
            return
        if data[:2] == b'\x10\x00' and len(data) >= 3:
            self._respond_calibration(data[2])
        elif data[0] == 0x0D:
            self.read_mode = data[1]
            if data[1] != 2:
                self.mode = data[1]
        elif data[0] == 0x01 and self.kind == 'core':
            self.state[1], self.state[4], self.state[8], self.state[9] = data[1], data[2], data[7], data[8]
        elif data[0] == 0x09 and self.kind == 'insole':
            self.state[1], self.state[8], self.state[9] = data[1], data[7], data[8]

    def _respond_calibration(self, sensor_index: int) -> None:
        """FW は INSOLE が圧力を含む mode（3 / 4）のときだけ応答する"""
        self.calibration_requests.append(sensor_index)
        callback = self.notify.get(CHARACTERISTIC_SENSOR_VALUES_UUID)
        if self.kind != 'insole' or self.calibrations is None or self.mode not in (3, 4) or callback is None:
            return
        entry = self.calibrations[sensor_index]
        func, coefficients = entry if entry is not None else (0, (1.0,) * 5)
        packet = calibration_packet(sensor_index, func, coefficients)
        asyncio.get_running_loop().create_task(callback(None, bytearray(packet)))

    async def start_notify(self, uuid: str, callback: Callable[..., Any]) -> None:
        await super().start_notify(uuid, callback)
        if uuid == CHARACTERISTIC_SENSOR_VALUES_UUID:
            self.tasks[uuid] = asyncio.get_running_loop().create_task(self._feed(callback))
        elif uuid == CHARACTERISTIC_STEP_ANALYSIS_UUID and self.kind == 'insole' and self.steps:
            self.tasks[uuid] = asyncio.get_running_loop().create_task(self._feed_steps(callback))

    async def stop_notify(self, uuid: str) -> None:
        await super().stop_notify(uuid)
        task = self.tasks.pop(uuid, None)
        if task:
            task.cancel()

    async def disconnect(self) -> bool:
        for task in self.tasks.values():
            task.cancel()
        self.tasks.clear()
        return await super().disconnect()

    async def _feed_steps(self, callback: Callable[..., Any]) -> None:
        """1 歩ぶんのパケットを 2 回ずつ送る（FW は取りこぼし対策で 2 回送る）"""
        for step in range(1, self.steps + 1):
            for packet in gait_step_packets(step) * 2:
                await callback(None, bytearray(packet))
            await asyncio.sleep(self.PACKET_INTERVAL_SEC)

    def _packet(self) -> bytes:
        self.serial = (self.serial + 1) % 65536
        acc = (0, 0, round(32768 / [2, 4, 8, 16][self.state[8]]))  # 静止時の 1G
        if self.kind == 'core':
            return build_packet_50((32767, 0, 0, 0), (0, 0, 0), acc, serial=self.serial)
        if self.mode == 1:
            return build_packet_50((16384, 0, 0, 0), (0, 0, 0), acc, serial=self.serial)
        if self.mode == 3:
            return build_packet_55((0, 0, 0), acc, (100,) * 6, serial=self.serial)
        return build_packet_56((16384, 0, 0, 0), (0, 0, 0), acc, (100,) * 6, serial=self.serial)

    async def _feed(self, callback: Callable[..., Any]) -> None:
        while self.is_connected:
            if self.read_mode != 2:
                await callback(None, bytearray(self._packet()))
            await asyncio.sleep(self.PACKET_INTERVAL_SEC)
