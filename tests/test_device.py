"""OrpheCoreInsole（接続・コマンド・配送）"""

import asyncio
import unittest

from helpers import (
    CORE_DEVICE_INFORMATION,
    INSOLE_DEVICE_INFORMATION,
    FakeClient,
    Recorder,
    build_packet_40,
    build_packet_50,
    build_packet_55,
    build_packet_56,
    build_step_analysis,
    fake_discover,
)

from orphe_core_insole import OrpheCoreInsole, UnsupportedOperationError, core_profile, insole_profile
from orphe_core_insole.uuids import (
    CHARACTERISTIC_DEVICE_INFORMATION_UUID,
    CHARACTERISTIC_SENSOR_VALUES_UUID,
    CHARACTERISTIC_STEP_ANALYSIS_UUID,
)

DEVICES = (("Keyboard", "AA"), ("INS-L", "BB"), ("CR-3", "CC"), (None, "DD"))


def make_device(profile=None, devices=DEVICES, **kwargs) -> tuple[OrpheCoreInsole, list[FakeClient]]:
    clients: list[FakeClient] = []

    def factory(ble_device):
        di = INSOLE_DEVICE_INFORMATION if (ble_device.name or "").startswith("INS") else CORE_DEVICE_INFORMATION
        client = FakeClient(ble_device, di)
        clients.append(client)
        return client

    device = OrpheCoreInsole(profile, verbose=False, discover=fake_discover(*devices),
                             client_factory=factory, **kwargs)
    # テストでは書き込み後の待ちを省く
    return device, clients


async def connected(profile=None, address=None, **kwargs) -> tuple[OrpheCoreInsole, FakeClient]:
    device, clients = make_device(profile, **kwargs)
    assert await device.connect(address)
    device.profile.write_wait_sec = 0
    clients[0].writes.clear()  # connect() の時刻合わせは数えない
    return device, clients[0]


class TestConnect(unittest.IsolatedAsyncioTestCase):

    async def test_auto_connects_first_orphe_core_insole_and_detects_kind(self):
        device, client = await connected()
        self.assertEqual(device.kind, 'insole')
        self.assertEqual(client.device.address, "BB")
        self.assertTrue(device.is_connected())

    async def test_explicit_profile_filters_by_name(self):
        device, client = await connected(core_profile())
        self.assertEqual(device.kind, 'core')
        self.assertEqual(client.device.address, "CC")

    async def test_address_without_name_is_core(self):
        device, client = await connected(address="DD")
        self.assertEqual(device.kind, 'core')
        self.assertEqual(client.device.address, "DD")

    async def test_not_found(self):
        device, clients = make_device(devices=(("Keyboard", "AA"),))
        self.assertFalse(await device.connect())
        self.assertEqual(device.kind, 'auto')
        self.assertEqual(clients, [])
        self.assertFalse(device.is_connected())

    async def test_scan(self):
        device, _ = make_device()
        results = await device.scan()
        self.assertEqual([(r.address, r.kind) for r in results], [("BB", 'insole'), ("CC", 'core')])
        everything = await device.scan(all_devices=True)
        self.assertEqual([r.kind for r in everything], [None, 'insole', 'core', None])
        self.assertEqual(everything[0].rssi, -50)

    async def test_disconnect_event_from_monitor(self):
        device, _ = await connected(monitor_interval_sec=0.01)
        disconnected = asyncio.Event()
        device.on('disconnect', disconnected.set)
        await device.disconnect()
        await asyncio.wait_for(disconnected.wait(), timeout=1)
        self.assertFalse(device.is_connected())

    async def test_commands_require_connection(self):
        device, _ = make_device()
        with self.assertRaisesRegex(RuntimeError, "Not connected"):
            await device.read_device_information()


class TestCommands(unittest.IsolatedAsyncioTestCase):

    async def test_core_acc_range_rewrites_device_information(self):
        device, client = await connected(core_profile())
        await device.set_acc_range(16)
        self.assertEqual(client.writes, [(CHARACTERISTIC_DEVICE_INFORMATION_UUID,
                                          bytes([1, 1, 200, 0, 1, 3, 4, 3, 3] + [0] * 11))])
        self.assertEqual(device.device_information.range.acc, 3)

    async def test_core_led_commands(self):
        device, client = await connected(core_profile())
        await device.set_led(1, 2)
        await device.set_led_brightness(10)
        await device.set_lr(0)
        self.assertEqual([w[:3] for _, w in client.writes], [b"\x02\x01\x02", b"\x01\x01\x0a", b"\x01\x00\xc8"])

    async def test_insole_commands(self):
        device, client = await connected()
        await device.set_gyro_range(500)
        await device.set_mount_position(2)
        await device.set_data_streaming_mode(3)
        gyro, mount, mode = (w for _, w in client.writes)
        self.assertEqual((gyro[0], gyro[8], len(gyro)), (0x09, 1, 20))
        self.assertEqual((mount[0], mount[1]), (0x09, 2))
        self.assertEqual(mode, b"\x0d\x03")

    async def test_invalid_values_raise_before_writing(self):
        device, client = await connected(core_profile())
        with self.assertRaises(ValueError):
            await device.set_acc_range(3)
        with self.assertRaises(ValueError):
            await device.set_led(1, 9)
        with self.assertRaises(ValueError):
            await device.set_led_brightness(256)
        self.assertEqual(client.writes, [])

    async def test_device_specific_commands(self):
        insole, _ = await connected()
        with self.assertRaises(UnsupportedOperationError):
            await insole.set_led(1, 0)
        core, _ = await connected(core_profile())
        with self.assertRaises(UnsupportedOperationError):
            await core.set_data_streaming_mode(4)
        with self.assertRaises(UnsupportedOperationError):
            await core.set_mount_position(1)

    async def test_write_without_device_information(self):
        device, _ = await connected()
        with self.assertRaisesRegex(RuntimeError, "Device information is not set"):
            await device.write_device_information()

    async def test_print_device_information(self):
        device, _ = await connected(insole_profile())
        from contextlib import redirect_stdout
        from io import StringIO
        out = StringIO()
        with redirect_stdout(out):
            await device.print_device_information()
        self.assertEqual(out.getvalue().splitlines(),
                         ["Version: 5", "Battery: 70", "Mount Position: 1", "ACC Range: 3", "GYRO Range: 3"])


class TestSensorValues(unittest.IsolatedAsyncioTestCase):

    async def test_insole_order_and_lost_data(self):
        device, client = await connected()
        recorder = Recorder()
        recorder.listen(device, 'acc', 'converted_acc', 'gyro', 'converted_gyro', 'pressure', 'quat', 'lost_data')
        await device.start_sensor_values_notification()
        handler = client.notify[CHARACTERISTIC_SENSOR_VALUES_UUID]

        packet = build_packet_56(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0), press=(1,) * 6, serial=1)
        await handler(None, bytearray(packet))
        per_frame = ['acc', 'converted_acc', 'gyro', 'converted_gyro', 'pressure', 'quat']
        self.assertEqual(recorder.names(), per_frame * 2)

        await handler(None, bytearray(build_packet_55(gyro=(0, 0, 0), acc=(0, 0, 0), press=(0,) * 6, serial=4)))
        self.assertEqual(recorder.of('lost_data'), [(1, 4)])
        self.assertEqual(device.serial_number_prev, 4)

    async def test_core_lost_data_only_for_header_50(self):
        device, client = await connected(core_profile())
        recorder = Recorder()
        recorder.listen(device, 'quat', 'lost_data')
        await device.start_sensor_values_notification()
        handler = client.notify[CHARACTERISTIC_SENSOR_VALUES_UUID]

        await handler(None, bytearray(build_packet_40(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0))))
        await handler(None, bytearray(build_packet_50(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0), serial=1)))
        await handler(None, bytearray(build_packet_50(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0), serial=3)))
        self.assertEqual(len(recorder.of('quat')), 1 + 4 + 4)
        self.assertEqual(recorder.of('lost_data'), [(1, 3)])

    async def test_first_packet_reports_gap_from_zero(self):
        # 公開版と同じく、直前のシリアル番号の初期値は 0
        device, _ = await connected()
        recorder = Recorder()
        recorder.listen(device, 'lost_data')
        await device.start_sensor_values_notification()
        packet = build_packet_56(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0), press=(0,) * 6, serial=10)
        device.handle_sensor_values(packet)
        self.assertEqual(recorder.of('lost_data'), [(0, 10)])

    async def test_ignored_after_disconnect(self):
        device, _ = await connected()
        recorder = Recorder()
        recorder.listen(device, 'acc')
        await device.start_sensor_values_notification()
        await device.disconnect()
        device.handle_sensor_values(build_packet_56(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0), press=(0,) * 6))
        self.assertEqual(recorder.events, [])

    async def test_on_returns_unsubscribe(self):
        device, _ = await connected()
        await device.read_device_information()
        received = []
        unsubscribe = device.on('pressure', received.append)
        packet = build_packet_56(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0), press=(0,) * 6)
        device.handle_sensor_values(packet)
        unsubscribe()
        device.handle_sensor_values(packet)
        self.assertEqual(len(received), 2)
        with self.assertRaisesRegex(ValueError, "unknown event"):
            device.on('press', print)


class TestStepAnalysis(unittest.IsolatedAsyncioTestCase):

    async def test_duplicates_are_dropped(self):
        device, client = await connected(core_profile())
        recorder = Recorder()
        recorder.listen(device, 'gait', 'stride', 'pronation', 'quat_distance')
        await device.start_step_analysis_notification()
        handler = client.notify[CHARACTERISTIC_STEP_ANALYSIS_UUID]

        for packet in (
            build_step_analysis(0, 1, flags=0b00001010, floats=(0.0, 1.5, 0.25, 0.5)),
            build_step_analysis(0, 1),  # 同じ歩数の 2 回目
            build_step_analysis(1, 1, floats=(10.0, 1.0, 2.0, 3.0)),
            build_step_analysis(2, 1),
            build_step_analysis(2, 1),
            build_step_analysis(4, 1),
            build_step_analysis(4, 1),  # クォータニオンは毎回通知
            build_step_analysis(3, 1),  # 未実装のサブヘッダ
        ):
            await handler(None, bytearray(packet))

        self.assertEqual(recorder.names(), ['gait', 'stride', 'pronation', 'quat_distance', 'quat_distance'])
        gait = recorder.of('gait')[0]
        self.assertEqual((gait.step_count, gait.gait_type, gait.direction), (1, 2, 1))
        self.assertEqual(gait.distance, (1.5,))
        self.assertEqual(recorder.of('stride')[0].foot_angle, (10.0,))
        self.assertEqual(device.step_count.quat, 1)


if __name__ == "__main__":
    unittest.main()
