"""FifoRecorder の収集ループ（FW を模擬した FifoClient を使う）"""

import asyncio
import unittest

from fifo_sim import FifoClient
from helpers import fake_discover

from orphe_core_insole import FifoRecorder, FifoTiming, OrpheCoreInsole
from orphe_core_insole.profiles import CoreProfile, InsoleProfile

FAST = FifoTiming(polling_interval_sec=0.05, retry_interval_sec=0.01, one_shot_idle_timeout_sec=0.1)


async def connect(**client_options) -> tuple[OrpheCoreInsole, FifoClient]:
    kind = client_options.get('kind', 'insole')
    client_options.setdefault('release_date', 20260905)
    profile = CoreProfile() if kind == 'core' else InsoleProfile()
    profile.write_wait_sec = 0
    clients = []

    def factory(ble_device):
        client = FifoClient(ble_device, **client_options)
        clients.append(client)
        return client

    name = "CR-3" if kind == 'core' else "INS-L"
    device = OrpheCoreInsole(profile, verbose=False, discover=fake_discover((name, "AA")), client_factory=factory)
    assert await device.connect()
    clients[0].writes.clear()  # connect() の時刻合わせは数えない
    return device, clients[0]


def recorder(device, **options) -> FifoRecorder:
    options.setdefault('startup_delay_sec', 0.05)
    options.setdefault('drain_timeout_sec', 0.5)
    options.setdefault('timing', FAST)
    return FifoRecorder(device, **options)


class Events:
    def __init__(self, fifo: FifoRecorder) -> None:
        self.samples = []
        self.losses = []
        self.stopped = []
        self.errors = []
        fifo.on('samples', self.samples.extend)
        fifo.on('data_loss', self.losses.append)
        fifo.on('stopped', self.stopped.append)
        fifo.on('error', self.errors.append)


class TestCollect(unittest.IsolatedAsyncioTestCase):

    async def collect(self, seconds=0.6, **client_options):
        device, client = await connect(**client_options)
        fifo = recorder(device)
        events = Events(fifo)
        self.assertTrue(await fifo.start(), events.errors)
        await asyncio.sleep(seconds)
        await fifo.stop()
        return device, client, fifo, events

    def assert_complete(self, fifo, client, events):
        serials = sorted(fifo.state.raw_store)
        self.assertGreater(len(serials), 10)
        self.assertEqual(serials, list(range(serials[0], serials[0] + len(serials))))  # 欠けなし
        # stop() 時点の FW の最新シリアル番号まで回収している（それ以降に作られたぶんは追わない）
        self.assertEqual(serials[-1], fifo._last_current_serial)
        self.assertEqual(fifo.dropped_count, 0)
        self.assertEqual(events.losses, [])
        self.assertEqual(events.stopped[0].reason, 'manual')
        self.assertEqual(events.errors, [])

    async def test_insole_4_samples(self):
        device, client, fifo, events = await self.collect()
        self.assert_complete(fifo, client, events)
        self.assertEqual(len(events.samples), 4 * fifo.collected_count)
        self.assertEqual(events.samples[0].pressure, tuple((events.samples[0].serial_number + j) % 4096
                                                           for j in range(6)))
        # 校正係数を読むため mode 4（圧力あり）→ FIFO の mode 2 → 終わったら streaming mode 4 に戻す。notify も止める
        self.assertEqual(client.mode_writes, [4, 2, 4])
        self.assertEqual(client.commands[-1], 0x06)
        self.assertFalse(device.sensor_values_notifying)
        csv = fifo.to_csv().splitlines()
        self.assertEqual(len(csv), 1 + 4 * fifo.collected_count)

    async def test_core_8x12_and_restores_realtime_mode(self):
        _device, client, fifo, events = await self.collect(kind='core')
        self.assert_complete(fifo, client, events)
        self.assertEqual(len(events.samples), 8 * fifo.collected_count)
        self.assertIsNone(events.samples[0].pressure)
        # CORE は 0x01（リアルタイム）に戻す。INSOLE の 0x04 を書くと実機が切断する
        self.assertEqual(client.mode_writes, [2, 1])
        self.assertEqual(fifo.to_csv().splitlines()[1].count(','), 7)

    async def test_restores_previous_streaming_mode(self):
        device, client = await connect()
        await device.set_data_streaming_mode(3)
        fifo = recorder(device)
        self.assertTrue(await fifo.start())
        await fifo.stop()
        self.assertEqual(client.mode_writes, [3, 2, 3])

    async def test_restore_mode_option(self):
        device, client = await connect()
        fifo = recorder(device, restore_mode=None)
        self.assertTrue(await fifo.start())
        await fifo.stop()
        self.assertEqual(client.mode_writes, [4, 2])  # 4 は校正係数を読むため

    async def test_short_max_write_sends_compact_requests(self):
        # 1 回で 104 バイトしか書けない → 122 バイトの要求は届かないので、終端の 0 の組を付けて短く送る
        _, client, fifo, events = await self.collect(kind='core', release_date=20260905, max_write=104,
                                                     drop=lambda serial, attempt: attempt == 1 and serial % 4 == 0)
        self.assert_complete(fifo, client, events)
        self.assertEqual(fifo.request_size, 104)
        requests = [data for _, data in client.writes if data[:2] == b'\x0b\x02']
        self.assertTrue(requests)
        self.assertTrue(all(len(data) <= 104 and data[-4:] == bytes(4) for data in requests))

    async def test_ble_loss_is_recovered(self):
        # 各パケットの最初の送信を 3 つに 1 つ落とす → 再要求で回収する
        _, client, fifo, events = await self.collect(drop=lambda serial, attempt: attempt == 1 and serial % 3 == 0)
        self.assert_complete(fifo, client, events)
        self.assertTrue(any(attempts > 1 for attempts in client.attempts.values()))

    async def test_fw_no_data_is_counted(self):
        # 最初の送信で落ちたパケットが、再要求の前に FW から消えた → fw_nodata
        device, client = await connect(drop=lambda serial, attempt: serial % 10 == 0)
        fifo = recorder(device)
        events = Events(fifo)

        original = client._respond_data

        def respond(data):
            packets = original(data)
            for serial in list(client.ring):
                if serial % 10 == 0 and client.attempts.get(serial, 0) >= 1:
                    del client.ring[serial]
            return packets

        client._respond_data = respond
        self.assertTrue(await fifo.start())
        await asyncio.sleep(0.6)
        await fifo.stop()
        self.assertGreater(fifo.dropped_count, 0)
        self.assertIn('fw_nodata', {loss.reason for loss in events.losses})
        # 範囲内の欠けはすべて dropped に入っている。新しい範囲の no-data で再同期すると同じシリアルを
        # もう一度要求して重ねて数えることがあるので、dropped は欠けの数以上になりうる
        span = max(fifo.state.raw_store) - min(fifo.state.raw_store) + 1
        self.assertGreaterEqual(fifo.dropped_count, span - fifo.collected_count)

    async def test_stop_on_loss(self):
        device, _client = await connect(drop=lambda serial, attempt: serial % 10 == 0, ring_capacity=5)
        fifo = recorder(device, stop_on_loss=True)
        events = Events(fifo)
        self.assertTrue(await fifo.start())
        for _ in range(40):
            if not fifo.is_running:
                break
            await asyncio.sleep(0.05)
        await fifo.stop()
        self.assertEqual(events.stopped[0].reason, 'loss')
        self.assertGreater(fifo.dropped_count, 0)

    async def test_catch_up_collects_backlog_after_stop(self):
        # ポーリングが遅いと stop() 時点で未要求のバックログが残る → catch-up で回収する
        device, client = await connect()
        fifo = recorder(device, timing=FifoTiming(polling_interval_sec=0.4, retry_interval_sec=0.01,
                                                  one_shot_idle_timeout_sec=0.1))
        events = Events(fifo)
        self.assertTrue(await fifo.start())
        await asyncio.sleep(0.7)
        await fifo.stop()
        self.assertGreater(events.stopped[0].catchup_recovered, 0)
        self.assert_complete_all(fifo, client)

    def assert_complete_all(self, fifo, client):
        serials = sorted(fifo.state.raw_store)
        self.assertEqual(serials, list(range(serials[0], serials[-1] + 1)))
        self.assertEqual(fifo.dropped_count, 0)


class TestStartFailures(unittest.IsolatedAsyncioTestCase):

    async def test_old_firmware_is_refused(self):
        device, client = await connect(kind='core', release_date=20260904)
        fifo = recorder(device)
        events = Events(fifo)
        self.assertFalse(await fifo.start())
        self.assertIn('20260905', str(events.errors[0]))
        self.assertEqual(client.writes, [])

    async def test_unknown_firmware_tries_anyway(self):
        device, _ = await connect(release_date=None)
        fifo = recorder(device)
        self.assertTrue(await fifo.start())
        await fifo.stop()

    async def test_not_connected(self):
        device, _ = await connect()
        await device.disconnect()
        fifo = recorder(device)
        events = Events(fifo)
        self.assertFalse(await fifo.start())
        self.assertIn('not connected', str(events.errors[0]))

    async def test_notify_already_taken(self):
        device, _client = await connect()
        device.set_notify_sink(lambda data: None)
        fifo = recorder(device)
        events = Events(fifo)
        self.assertFalse(await fifo.start())
        self.assertIn('already taken', str(events.errors[0]))

    async def test_no_ack_gives_up_and_restores(self):
        device, client = await connect(ack_drop={0x06: 100})
        fifo = recorder(device)
        events = Events(fifo)
        self.assertFalse(await fifo.start())
        self.assertIn('did not respond', str(events.errors[0]))
        self.assertEqual(client.mode_writes, [4, 2, 4])  # 最初の 4 は校正係数を読むため
        self.assertEqual(device._notify_sinks, {})

    async def test_stop_monitor_is_retried(self):
        device, client = await connect()
        fifo = recorder(device)
        self.assertTrue(await fifo.start())
        client.ack_drop[0x06] = 2
        await fifo.stop()
        self.assertEqual(client.commands[-3:], [0x06, 0x06, 0x06])
        self.assertFalse(client.monitoring)

    async def test_restart_resets_state(self):
        device, _ = await connect()
        fifo = recorder(device)
        self.assertTrue(await fifo.start())
        await asyncio.sleep(0.2)
        await fifo.stop()
        first = set(fifo.state.raw_store)
        self.assertTrue(await fifo.start())
        await asyncio.sleep(0.2)
        await fifo.stop()
        self.assertFalse(first & set(fifo.state.raw_store))

    async def test_realtime_events_resume_after_fifo(self):
        device, _client = await connect()
        received = []
        device.on('sensor_values', received.append)
        await device.start_sensor_values_notification()
        fifo = recorder(device)
        self.assertTrue(await fifo.start())
        await fifo.stop()
        # FIFO 前に始めた notify は止めない。横取りが外れ、リアルタイムのイベントがまた届く
        self.assertTrue(device.sensor_values_notifying)
        self.assertEqual(device._notify_sinks, {})
        before = len(received)
        await asyncio.sleep(0.1)
        self.assertGreater(len(received), before)
        self.assertTrue(all(packet.header == 56 for packet in received))  # FIFO のパケットは混ざらない


if __name__ == "__main__":
    unittest.main()
