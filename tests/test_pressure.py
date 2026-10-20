"""INSOLE の個体別圧力校正係数（換算・取得・converted_pressure・FIFO の CSV）"""

import asyncio
import math
import unittest

from fifo_sim import FifoClient
from helpers import SimulatedClient, build_packet_56, calibration_packet, fake_discover

from orphe_core_insole import FifoRecorder, FifoTiming, OrpheCoreInsole, UnsupportedOperationError
from orphe_core_insole.pressure import (
    PressureCalibration,
    apply_pressure_calibration,
    decode_pressure_calibration,
    encode_pressure_calibration_request,
    legacy_pressure_to_newton,
    pressure_to_newton,
)
from orphe_core_insole.profiles import CoreProfile, InsoleProfile

POLY = PressureCalibration(1, (1e-12, -1e-9, 1e-6, 0.01, 0.5))
EXP = PressureCalibration(0, (2.0, 0.001, 3.0, 0.0, 0.0))
# ch 1 と 4 は未書き込み（プレースホルダが返る）
CALIBRATIONS = [(1, POLY.coefficients), None, (0, EXP.coefficients), (1, POLY.coefficients), None,
                (0, EXP.coefficients)]


class TestConversion(unittest.TestCase):

    def test_codec(self):
        self.assertEqual(encode_pressure_calibration_request(5), bytes([0x10, 0x00, 5]))
        with self.assertRaises(ValueError):
            encode_pressure_calibration_request(6)
        self.assertEqual(decode_pressure_calibration(calibration_packet(3, 1, POLY.coefficients)), (3, POLY))
        self.assertIsNone(decode_pressure_calibration(calibration_packet(3, 1, POLY.coefficients)[:-1]))
        self.assertIsNone(decode_pressure_calibration(bytes([0x36]) + bytes(42)))

    def test_apply(self):
        x = 1000
        self.assertAlmostEqual(apply_pressure_calibration(POLY, x), 1e-12 * x ** 4 - 1e-9 * x ** 3 + 1e-6 * x ** 2
                               + 0.01 * x + 0.5)
        self.assertAlmostEqual(apply_pressure_calibration(EXP, x), 2.0 * math.exp(1.0) + 3.0)
        self.assertEqual(apply_pressure_calibration(PressureCalibration(0, (-5.0, 0.0, 1.0)), x), 0)  # 負は 0
        self.assertEqual(apply_pressure_calibration(PressureCalibration(0, (1.0, 1e6, 0.0)), x), 0)   # 無限大は 0
        self.assertEqual(apply_pressure_calibration(PressureCalibration(2, (1.0,) * 5), x), 0)       # 未知の式

    def test_legacy_and_fallback(self):
        self.assertEqual(legacy_pressure_to_newton(297), 0)  # 239.3mV 以下は 0
        mv = 2000 * 3.3 / 4096 * 1000
        self.assertAlmostEqual(legacy_pressure_to_newton(2000), 2.77942 * math.exp(2.08348e-3 * mv) + 4.14411)
        placeholder = PressureCalibration(0, (1.0,) * 5)
        self.assertTrue(placeholder.is_placeholder)
        self.assertEqual(pressure_to_newton(2000, placeholder), legacy_pressure_to_newton(2000))
        self.assertEqual(pressure_to_newton(2000, None), legacy_pressure_to_newton(2000))
        self.assertEqual(pressure_to_newton(2000, POLY), apply_pressure_calibration(POLY, 2000))


async def connect(kind='insole', client_class=SimulatedClient, **options):
    profile = CoreProfile() if kind == 'core' else InsoleProfile()
    profile.write_wait_sec = 0
    clients = []

    def factory(ble_device):
        client = (client_class(ble_device, kind, **options) if client_class is SimulatedClient
                  else client_class(ble_device, kind=kind, **options))
        clients.append(client)
        return client

    name = "CR-3" if kind == 'core' else "INS-L"
    device = OrpheCoreInsole(profile, verbose=False, discover=fake_discover((name, "AA")), client_factory=factory)
    assert await device.connect()
    return device, clients[0]


class TestRead(unittest.IsolatedAsyncioTestCase):

    async def test_reads_six_channels(self):
        device, client = await connect(release_date=20260428, calibrations=CALIBRATIONS)
        events = []
        device.on('acc', events.append)
        result = await device.read_pressure_calibrations(timeout_sec=0.2)
        self.assertEqual(result, [POLY, None, EXP, POLY, None, EXP])
        self.assertIs(device.pressure_calibrations, result)
        self.assertEqual(client.calibration_requests, [0, 1, 2, 3, 4, 5])
        # notify は読む間だけ購読し、そのあいだセンサー値のイベントは出さない
        self.assertFalse(device.sensor_values_notifying)
        self.assertEqual(events, [])

    async def test_switches_to_pressure_mode_and_back(self):
        device, client = await connect(release_date=20260428, calibrations=CALIBRATIONS)
        await device.set_data_streaming_mode(1)
        await device.read_pressure_calibrations(timeout_sec=0.2)
        self.assertEqual(device.pressure_calibrations[0], POLY)
        self.assertEqual([data[1] for _, data in client.writes if data[0] == 0x0D], [1, 4, 1])

    async def test_no_response_channels_are_none(self):
        device, client = await connect(release_date=20260428, calibrations=None)
        result = await device.read_pressure_calibrations(timeout_sec=0.05, retries=2)
        self.assertEqual(result, [None] * 6)
        self.assertEqual(len(client.calibration_requests), 12)

    async def test_old_or_unknown_firmware(self):
        for release_date in (20260427, None):
            device, client = await connect(release_date=release_date, calibrations=CALIBRATIONS)
            self.assertIsNone(await device.read_pressure_calibrations(timeout_sec=0.1))
            self.assertIsNone(device.pressure_calibrations)
            self.assertEqual(client.calibration_requests, [])

    async def test_core_is_unsupported(self):
        device, _ = await connect(kind='core', release_date=20260905)
        with self.assertRaises(UnsupportedOperationError):
            await device.read_pressure_calibrations()


class TestConvertedPressure(unittest.IsolatedAsyncioTestCase):

    async def test_event_uses_calibrations(self):
        device, _ = await connect(release_date=20260428, calibrations=CALIBRATIONS)
        await device.read_pressure_calibrations(timeout_sec=0.2)
        await device.read_device_information()
        received = []
        device.on('converted_pressure', received.append)
        raw = (100, 2000, 1500, 3000, 400, 2500)
        device.handle_sensor_values(build_packet_56((16384, 0, 0, 0), (0, 0, 0), (0, 0, 0), raw, serial=7))
        self.assertEqual(len(received), 2)
        expected = [pressure_to_newton(v, c) for v, c in zip(raw, device.pressure_calibrations)]
        self.assertEqual(received[0].values, expected)
        self.assertEqual((received[0].serial_number, received[1].packet_number), (7, 1))

    async def test_without_calibrations_uses_legacy_formula(self):
        device, _ = await connect()
        await device.read_device_information()
        received = []
        device.on('converted_pressure', received.append)
        device.handle_sensor_values(build_packet_56((0, 0, 0, 0), (0, 0, 0), (0, 0, 0), (2000,) * 6))
        self.assertEqual(received[0].values, [legacy_pressure_to_newton(2000)] * 6)


class TestFifoCsv(unittest.IsolatedAsyncioTestCase):

    async def test_csv_uses_calibrations(self):
        device, _client = await connect(client_class=FifoClient, release_date=20260510, calibrations=CALIBRATIONS)
        fifo = FifoRecorder(device, startup_delay_sec=0.05, drain_timeout_sec=0.3,
                            timing=FifoTiming(polling_interval_sec=0.05, retry_interval_sec=0.01,
                                              one_shot_idle_timeout_sec=0.1))
        self.assertTrue(await fifo.start())
        await asyncio.sleep(0.3)
        await fifo.stop()
        self.assertEqual(device.pressure_calibrations, [POLY, None, EXP, POLY, None, EXP])
        row = fifo.to_csv().splitlines()[1].split(', ')
        serial = int(row[0])
        raw = [(serial + j) % 4096 for j in range(6)]  # fifo_sim の圧力の生値
        expected = [f"{pressure_to_newton(v, c):.4f}" for v, c in zip(raw, device.pressure_calibrations)]
        self.assertEqual([column.strip() for column in row[8:]], expected)


if __name__ == "__main__":
    unittest.main()
