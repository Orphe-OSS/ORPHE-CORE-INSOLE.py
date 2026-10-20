"""デバイス時計の同期（DATE_TIME）"""

# デバイスの時計はローカル時刻（タイムゾーンなし）
# ruff: noqa: DTZ001, DTZ005

import unittest
from datetime import datetime

from helpers import FakeClient, fake_discover

from orphe_core_insole import OrpheCoreInsole
from orphe_core_insole.time_sync import decode_date_time, encode_date_time, read_date_time, sync_device_time
from orphe_core_insole.uuids import CHARACTERISTIC_DATE_TIME_UUID


class TestWireFormat(unittest.TestCase):

    def test_encode_and_decode(self):
        date = datetime(2026, 9, 30, 14, 5, 7, 123456)
        self.assertEqual(encode_date_time(date), bytes([26, 9, 30, 14, 5, 7, 12]))
        self.assertEqual(decode_date_time(bytes([26, 9, 30, 14, 5, 7, 12])), datetime(2026, 9, 30, 14, 5, 7, 120000))

    def test_decode_invalid_value_is_none(self):
        # 時計が未設定で月・日が 0 のときなど
        self.assertIsNone(decode_date_time(bytes(7)))
        self.assertIsNone(decode_date_time(bytes([26, 9, 30])))


class Clock:
    """呼ぶたびに steps の値だけ進む単調クロック[s]"""

    def __init__(self, *steps: float) -> None:
        self.now = 0.0
        self.steps = list(steps)

    def __call__(self) -> float:
        value = self.now
        if self.steps:
            self.now += self.steps.pop(0)
        return value


class TestSync(unittest.IsolatedAsyncioTestCase):

    async def test_read_measures_round_trip_time(self):
        async def read():
            return bytes([26, 1, 2, 3, 4, 5, 6])

        got = await read_date_time(read, clock=Clock(0.0125))
        self.assertEqual(got.round_trip_time, 12)  # 切り捨て
        self.assertEqual(got.date, datetime(2026, 1, 2, 3, 4, 5, 60000))
        self.assertEqual(got.raw, bytes([26, 1, 2, 3, 4, 5, 6]))

    async def test_writes_pc_time_plus_half_round_trip(self):
        written = []

        async def read():
            return bytes(7)  # デバイスの時計が未設定でも合わせられる

        async def write(data):
            written.append(data)

        # 往復時間 10, 20, 31ms → 平均 20.33ms → 片道 10ms
        clock = Clock(0.0105, 0, 0.0205, 0, 0.0315, 0)
        now = datetime(2026, 9, 30, 12, 0, 0, 995000)
        result = await sync_device_time(read, write, samples=3, now=lambda: now, clock=clock)
        self.assertEqual(result.round_trip_times, [10, 20, 31])
        self.assertEqual(result.sum_round_trip_time, 61)
        self.assertAlmostEqual(result.average_round_trip_time, 61 / 3)
        self.assertEqual(result.half_round_trip_time, 10)
        self.assertEqual(result.adjusted_time - result.standard_time, 10)
        # 12:00:00.995 + 10ms = 12:00:01.005
        self.assertEqual(written, [bytes([26, 9, 30, 12, 0, 1, 0])])

    async def test_half_round_trip_rounds_half_up(self):
        async def read():
            return bytes(7)

        async def write(data):
            pass

        result = await sync_device_time(read, write, samples=1, clock=Clock(0.0055, 0))
        self.assertEqual(result.half_round_trip_time, 3)  # 2.5 → 3

    async def test_samples_must_be_positive(self):
        with self.assertRaises(ValueError):
            await sync_device_time(None, None, samples=0)  # type: ignore[arg-type]


class TimeClient(FakeClient):
    def __init__(self, device, fail_read=False):
        super().__init__(device)
        self.date_time_reads = 0
        self.fail_read = fail_read

    async def read_gatt_char(self, uuid):
        if uuid == CHARACTERISTIC_DATE_TIME_UUID:
            self.date_time_reads += 1
            if self.fail_read:
                raise RuntimeError("read failed")
        return await super().read_gatt_char(uuid)


def make_device(fail_read=False, **kwargs):
    clients = []

    def factory(ble_device):
        client = TimeClient(ble_device, fail_read=fail_read)
        clients.append(client)
        return client

    device = OrpheCoreInsole(verbose=False, discover=fake_discover(("CR-3", "AA")), client_factory=factory, **kwargs)
    return device, clients


class TestConnect(unittest.IsolatedAsyncioTestCase):

    async def test_connect_syncs_device_time(self):
        device, clients = make_device()
        before = datetime.now()
        self.assertTrue(await device.connect())
        client = clients[0]
        self.assertEqual(client.date_time_reads, 3)
        self.assertEqual([uuid for uuid, _ in client.writes], [CHARACTERISTIC_DATE_TIME_UUID])
        self.assertEqual(len(device.time_sync.round_trip_times), 3)
        written = decode_date_time(client.writes[0][1])
        self.assertLess(abs((written - before).total_seconds()), 5)

    async def test_time_sync_samples(self):
        device, clients = make_device(time_sync_samples=5)
        await device.connect()
        self.assertEqual(clients[0].date_time_reads, 5)

        device, clients = make_device(time_sync_samples=0)
        await device.connect()
        self.assertEqual(clients[0].date_time_reads, 0)
        self.assertEqual(clients[0].writes, [])
        self.assertIsNone(device.time_sync)

    async def test_sync_failure_disconnects(self):
        device, clients = make_device(fail_read=True)
        with self.assertRaises(RuntimeError):
            await device.connect()
        self.assertFalse(clients[0].is_connected)

    async def test_read_and_write_date_time(self):
        device, _ = make_device(time_sync_samples=0)
        await device.connect()
        await device.write_date_time(datetime(2027, 1, 2, 3, 4, 5, 60000))
        got = await device.read_date_time()
        self.assertEqual(got.date, datetime(2027, 1, 2, 3, 4, 5, 60000))

        result = await device.sync_device_time(samples=2)
        self.assertEqual(len(result.round_trip_times), 2)
