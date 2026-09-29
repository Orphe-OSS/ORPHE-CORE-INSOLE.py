"""
公開版 ORPHE-CORE.py / ORPHE-INSOLE.py との一致確認

公開版のリポジトリを隣に置いたとき（または ORPHE_LEGACY_DIR で場所を指定したとき）だけ動く。
    ORPHE_LEGACY_DIR/ORPHE-CORE.py/orphe_core.py
    ORPHE_LEGACY_DIR/ORPHE-INSOLE.py/orphe_insole.py

ランダムなパケットとコマンドを両方に通し、コールバックに渡る値・書き込むバイト列・
標準出力が一致することを確かめる。
"""

from __future__ import annotations

import importlib.util
import os
import random
import sys
import types
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from typing import Any
from unittest import mock

from helpers import CORE_DEVICE_INFORMATION, INSOLE_DEVICE_INFORMATION, FakeClient

from orphe_core_insole.compat import orphe_core as compat_core
from orphe_core_insole.compat import orphe_insole as compat_insole

LEGACY_DIR = Path(os.environ.get('ORPHE_LEGACY_DIR', Path(__file__).resolve().parents[2]))


def load_legacy(repo: str, module: str) -> types.ModuleType | None:
    path = LEGACY_DIR / repo / f'{module}.py'
    if not path.exists():
        return None
    if 'bleak' not in sys.modules and importlib.util.find_spec('bleak') is None:
        stub = types.ModuleType('bleak')
        stub.BleakClient = object
        stub.BleakScanner = object
        sys.modules['bleak'] = stub
    spec = importlib.util.spec_from_file_location(f'legacy_{module}', path)
    legacy = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(legacy)
    return legacy


LEGACY_CORE = load_legacy('ORPHE-CORE.py', 'orphe_core')
LEGACY_INSOLE = load_legacy('ORPHE-INSOLE.py', 'orphe_insole')

CALLBACKS = (
    'got_acc_callback', 'got_converted_acc_callback', 'got_gyro_callback',
    'got_converted_gyro_callback', 'got_pressure_callback', 'got_quat_callback',
    'got_gait_callback', 'got_stride_callback', 'got_pronation_callback',
    'got_quat_distance_callback', 'lost_data_callback',
)


def snapshot(value: Any) -> Any:
    """比較用に値を repr できる形にする（NaN も一致とみなすため repr で比べる）"""
    if isinstance(value, (bytes, bytearray)):
        return bytes(value)
    if hasattr(value, '__dict__') and not isinstance(value, type):
        return (type(value).__name__,
                tuple(sorted((k, repr(snapshot(v))) for k, v in vars(value).items()
                             if k not in ('owner',))))
    return value


class Owner:
    """公開版の SensorValuesData / StepAnalysisData に渡すオーナー役"""

    def __init__(self, step_count: Any) -> None:
        self.events: list[tuple[str, Any]] = []
        self.step_count = step_count
        for name in CALLBACKS:
            setattr(self, name, self._recorder(name))

    def _recorder(self, name: str):
        return lambda *args: self.events.append((name, repr([snapshot(a) for a in args])))


def random_sensor_packet(rng: random.Random, header: int) -> bytearray:
    data = bytearray(rng.randrange(256) for _ in range(104))
    data[0] = header
    ms = rng.randrange(1000)
    data[3:8] = bytes([rng.randrange(24), rng.randrange(60), rng.randrange(60), ms >> 8, ms & 0xFF])
    return data


def random_step_packet(rng: random.Random) -> bytearray:
    data = bytearray(rng.randrange(256) for _ in range(20))
    data[1] = rng.randrange(7)
    data[2:4] = rng.randrange(6).to_bytes(2, 'big')
    return data


class ParityMixin:
    legacy: types.ModuleType | None
    compat: types.ModuleType
    headers: tuple[int, ...]
    device_information: bytes

    def setUp(self) -> None:
        if self.legacy is None:
            self.skipTest(f'公開版が見つからない（{LEGACY_DIR}）')

    def test_sensor_values(self) -> None:
        rng = random.Random(1)
        for _ in range(3000):
            data = random_sensor_packet(rng, rng.choice(self.headers))
            sensor_range = self.legacy.Range()
            sensor_range.acc, sensor_range.gyro = rng.randrange(4), rng.randrange(4)
            expected, actual = Owner(None), Owner(None)
            legacy_values = self.legacy.SensorValuesData(expected, data, sensor_range)
            compat_values = self.compat.SensorValuesData(actual, data, sensor_range)
            self.assertEqual(actual.events, expected.events, f'header {data[0]}')
            self.assertEqual(snapshot(compat_values)[1], snapshot(legacy_values)[1])

    def test_step_analysis(self) -> None:
        rng = random.Random(2)
        expected, actual = Owner(self.legacy.StepCount()), Owner(self.compat.StepCount())
        for _ in range(3000):
            data = random_step_packet(rng)
            legacy_values = self.legacy.StepAnalysisData(expected, data)
            compat_values = self.compat.StepAnalysisData(actual, data)
            self.assertEqual(actual.events, expected.events)
            self.assertEqual(snapshot(compat_values)[1], snapshot(legacy_values)[1])
            self.assertEqual(vars(actual.step_count), vars(expected.step_count))

    def _pair(self) -> tuple[Any, Any]:
        legacy = self.legacy.Orphe()
        legacy.client = FakeClient(device_information=self.device_information)
        legacy.client.is_connected = True
        compat = self.compat.Orphe(verbose=False)
        compat.client = FakeClient(device_information=self.device_information)
        compat.client.is_connected = True
        compat.profile.write_wait_sec = 0
        return legacy, compat

    async def _run(self, orphe: Any, script: list[tuple[str, tuple[Any, ...]]]) -> str:
        out = StringIO()
        with redirect_stdout(out), mock.patch.object(self.legacy, 'WRITE_WAIT_INTERVAL_SEC', 0):
            for name, args in script:
                await getattr(orphe, name)(*args)
        return out.getvalue()

    async def _assert_same_commands(self, script: list[tuple[str, tuple[Any, ...]]]) -> None:
        legacy, compat = self._pair()
        self.assertEqual(await self._run(compat, script), await self._run(legacy, script))
        self.assertEqual(compat.client.writes, legacy.client.writes)

    async def test_notifications(self) -> None:
        rng = random.Random(3)
        legacy, compat = self._pair()
        recorders = []
        for orphe in (legacy, compat):
            owner = Owner(None)
            recorders.append(owner)
            for name in CALLBACKS:
                setter = f'set_{name}'
                if hasattr(orphe, setter):
                    getattr(orphe, setter)(getattr(owner, name))
            await self._run(orphe, [('start_sensor_values_notification', ()),
                                    ('start_step_analysis_notification', ())])

        serial = 0
        for _ in range(500):
            data = random_sensor_packet(rng, rng.choice(self.headers))
            serial = (serial + rng.choice((1, 1, 1, 2, 5))) % 65536
            data[1:3] = serial.to_bytes(2, 'big')
            step = random_step_packet(rng)
            for orphe in (legacy, compat):
                await orphe.sensor_values_notification_handler(None, data)
                await orphe.step_analysis_notification_handler(None, step)
        self.assertEqual(recorders[1].events, recorders[0].events)
        self.assertEqual(compat.serial_number_prev, legacy.serial_number_prev)


@unittest.skipIf(LEGACY_CORE is None, '公開版 ORPHE-CORE.py が見つからない')
class TestCoreParity(ParityMixin, unittest.IsolatedAsyncioTestCase):
    legacy = LEGACY_CORE
    compat = compat_core
    headers = (40, 50, 55, 56, 0)
    device_information = CORE_DEVICE_INFORMATION

    async def test_commands(self) -> None:
        await self._assert_same_commands([
            ('print_device_information', ()),
            ('set_led', (1, 2)),
            ('set_led', (2, 0)),
            ('set_led', (1, 7)),
            ('set_led_brightness', (128,)),
            ('set_led_brightness', (300,)),
            ('set_lr', (0,)),
            ('set_lr', (3,)),
            ('set_acc_range', (8,)),
            ('set_acc_range', (3,)),
            ('set_gyro_range', (1000,)),
            ('set_gyro_range', (1,)),
            ('write_device_information', (bytearray([1] * 20),)),
        ])


@unittest.skipIf(LEGACY_INSOLE is None, '公開版 ORPHE-INSOLE.py が見つからない')
class TestInsoleParity(ParityMixin, unittest.IsolatedAsyncioTestCase):
    legacy = LEGACY_INSOLE
    compat = compat_insole
    headers = (40, 50, 55, 56, 0)
    device_information = INSOLE_DEVICE_INFORMATION

    async def test_commands(self) -> None:
        await self._assert_same_commands([
            ('print_device_information', ()),
            ('set_mount_position', (2,)),
            ('set_mount_position', (5,)),
            ('set_acc_range', (16,)),
            ('set_acc_range', (5,)),
            ('set_gyro_range', (500,)),
            ('set_data_streaming_mode', (3,)),
            ('set_data_streaming_mode', (2,)),
            ('set_data_streaming_mode', ()),
        ])

    async def test_checksum_helpers(self) -> None:
        legacy, compat = self._pair()
        data = bytes(range(40, 60))
        self.assertEqual(await compat.getChecksum(data), await legacy.getChecksum(data))
        for value in (0, 7, 255, 4095):
            self.assertEqual(await compat.right_dec2hex(value), await legacy.right_dec2hex(value))


if __name__ == '__main__':
    unittest.main()
