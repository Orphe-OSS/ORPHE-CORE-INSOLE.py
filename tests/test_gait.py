"""INSOLE の歩行解析（STEP_ANALYSIS）"""

import asyncio
import struct
import unittest

from helpers import INSOLE_DEVICE_INFORMATION, FakeClient, fake_discover

from orphe_core_insole import OrpheCoreInsole, insole_profile
from orphe_core_insole.gait import (
    GAIT_CSV_HEADER,
    GaitAggregator,
    GaitIncompleteStep,
    GaitMotionPacket,
    GaitOverviewPacket,
    GaitPronationPacket,
    GaitStepGap,
    GaitStepNumberJump,
    GaitStridePacket,
    InsoleGait,
    decode_gait_packet,
    gait_row_to_csv,
)
from orphe_core_insole.gait.aggregator import build_gait_row, step_distance
from orphe_core_insole.uuids import CHARACTERISTIC_STEP_ANALYSIS_UUID


def f32(value: float) -> bytes:
    return struct.pack('>f', value)


def u16(value: int) -> bytes:
    return value.to_bytes(2, 'big')


def overview_packet(step, gait_type=1, direction=1, calorie_bits=0x3E00, distance=12.5, stance=0.5, swing=0.25):
    flags = (gait_type << 6) | (direction << 3)
    return bytes([51, 0]) + u16(step) + bytes([flags, 0]) + u16(calorie_bits) + f32(distance) + f32(stance) + f32(swing)


def stride_packet(step, foot_angle=5.5, x=3, y=4, z=0):
    return bytes([51, 1]) + u16(step) + f32(foot_angle) + f32(x) + f32(y) + f32(z)


def pronation_packet(step, landing=1.25, px=-5, py=-9.4000005, pz=2):
    return bytes([51, 2]) + u16(step) + f32(landing) + f32(px) + f32(py) + f32(pz)


def motion_packet(step):
    # quat w=1、delta 0.5, 1, 2（float16）
    return (bytes([51, 4]) + u16(step) + bytes([0b01001010, 0, 0x3C, 0, 0, 0, 0, 0, 0, 0])
            + bytes([0x38, 0, 0x3C, 0, 0x40, 0]))


def step_packets(step):
    return [overview_packet(step), stride_packet(step), pronation_packet(step)]


class TestDecode(unittest.TestCase):

    def test_decodes_each_subheader(self):
        overview = decode_gait_packet(overview_packet(7))
        self.assertIsInstance(overview, GaitOverviewPacket)
        self.assertEqual(overview.step_number, 7)
        self.assertEqual((overview.gait_type, overview.stride_direction), ('walk', 'forward'))
        self.assertEqual(overview.calorie, 1.5)
        self.assertEqual((overview.distance_m, overview.stance_phase_s, overview.swing_phase_s), (12.5, 0.5, 0.25))

        stride = decode_gait_packet(stride_packet(7))
        self.assertIsInstance(stride, GaitStridePacket)
        self.assertEqual((stride.foot_angle, stride.stride_x, stride.stride_y, stride.stride_z), (5.5, 3, 4, 0))

        pronation = decode_gait_packet(pronation_packet(7))
        self.assertIsInstance(pronation, GaitPronationPacket)
        self.assertEqual((pronation.landing_force, pronation.pronation_x), (1.25, -5))

        motion = decode_gait_packet(motion_packet(7))
        self.assertIsInstance(motion, GaitMotionPacket)
        self.assertEqual((motion.gait_cycle_phase, motion.gait_cycle_period, motion.gait_cycle_event), (1, 1, 2))
        self.assertEqual((motion.quat_w, motion.delta_x, motion.delta_y, motion.delta_z), (1, 0.5, 1, 2))

    def test_negative_sentinel_is_none_only_for_non_negative_values(self):
        overview = decode_gait_packet(overview_packet(1, stance=-1, distance=-1))
        self.assertIsNone(overview.stance_phase_s)
        self.assertIsNone(overview.distance_m)
        self.assertEqual(overview.swing_phase_s, 0.25)
        self.assertEqual(decode_gait_packet(stride_packet(1, foot_angle=-12.5)).foot_angle, -12.5)
        self.assertIsNone(decode_gait_packet(pronation_packet(1, landing=-1)).landing_force)
        self.assertIsNone(decode_gait_packet(stride_packet(1, x=float('nan'))).stride_x)

    def test_not_gait_packet_is_none(self):
        self.assertIsNone(decode_gait_packet(bytes(19)))
        self.assertIsNone(decode_gait_packet(bytes([50]) + overview_packet(1)[1:]))
        self.assertIsNone(decode_gait_packet(bytes([51, 3]) + overview_packet(1)[2:]))


def feed(aggregator, data):
    packet = decode_gait_packet(data)
    if packet is None or isinstance(packet, GaitMotionPacket):
        return None
    return aggregator.add(packet)


def row_for(**overview):
    return build_gait_row(5, decode_gait_packet(overview_packet(5, **overview)),
                          decode_gait_packet(stride_packet(5)), decode_gait_packet(pronation_packet(5)))


class TestAggregator(unittest.TestCase):

    def test_row_only_when_three_parts_arrive_and_duplicates_are_ignored(self):
        aggregator = GaitAggregator()
        self.assertIsNone(feed(aggregator, overview_packet(10)))
        self.assertIsNone(feed(aggregator, stride_packet(10)))
        row = feed(aggregator, pronation_packet(10))
        self.assertEqual(row.step_number, 10)
        self.assertEqual(row.gait_type, 'walk')
        self.assertEqual(row.duration_s, 0.75)
        self.assertEqual(row.cadence_hz, 1 / 0.75)
        self.assertEqual(row.stride_norm_m, 5)
        self.assertEqual(row.speed_mps, 5 / 0.75)
        self.assertEqual(row.foot_strike, 'heelStrike')
        self.assertEqual(row.pronation_type, 'neutral')

        # 2 回目の送信は無視する
        self.assertIsNone(feed(aggregator, overview_packet(10)))
        self.assertIsNone(feed(aggregator, pronation_packet(10)))
        self.assertEqual(aggregator.stats().completed_steps, 1)

    def test_gap_and_late_arrival(self):
        aggregator = GaitAggregator()
        losses = []
        aggregator.on_step_loss = losses.append
        for data in step_packets(1):
            feed(aggregator, data)
        feed(aggregator, overview_packet(3))  # step 2 が飛んだ
        self.assertEqual(aggregator.stats().gap_steps, 1)
        self.assertEqual(losses[0], GaitStepGap(steps=(2,), count=1))

        # 遅れて届いたら数え直す
        feed(aggregator, overview_packet(2))
        self.assertEqual(aggregator.stats().gap_steps, 0)

    def test_stale_step_is_incomplete_and_big_forward_is_jump(self):
        aggregator = GaitAggregator()
        feed(aggregator, overview_packet(1))
        feed(aggregator, overview_packet(9))  # 1 は 8 歩以上古い
        stats = aggregator.stats()
        self.assertEqual(stats.incomplete_steps, 1)
        self.assertEqual((stats.missing_parts.overview, stats.missing_parts.stride, stats.missing_parts.pronation),
                         (0, 1, 1))

        feed(aggregator, overview_packet(9 + 1000))
        self.assertEqual(aggregator.stats().jumps, 1)

    def test_wraparound_gap_and_jump(self):
        aggregator = GaitAggregator()
        losses = []
        aggregator.on_step_loss = losses.append
        rows = []
        for step in (65534, 1):
            for data in step_packets(step):
                row = feed(aggregator, data)
                if row:
                    rows.append(row.step_number)
        feed(aggregator, overview_packet(1 + 1000))

        self.assertEqual(rows, [65534, 1])
        self.assertEqual(losses, [
            GaitStepGap(steps=(65535, 0), count=2),
            GaitStepNumberJump(from_step=1, to_step=1001, forward=1000),
        ])
        stats = aggregator.stats()
        self.assertEqual((stats.completed_steps, stats.incomplete_steps, stats.gap_steps, stats.jumps),
                         (2, 0, 2, 1))
        self.assertEqual((stats.last_seen_step, stats.pending_steps), (1001, 1))

    def test_incomplete_reports_missing_parts(self):
        aggregator = GaitAggregator()
        losses = []
        aggregator.on_step_loss = losses.append
        feed(aggregator, stride_packet(1))
        feed(aggregator, overview_packet(9))
        self.assertEqual(losses, [
            GaitStepGap(steps=(2, 3, 4, 5, 6, 7, 8), count=7),
            GaitIncompleteStep(step_number=1, missing=('overview', 'pronation')),
        ])

    def test_step_distance(self):
        self.assertEqual(step_distance(65534, 2), 4)
        self.assertEqual(step_distance(2, 65534), 65532)
        self.assertEqual(step_distance(5, 5), 0)

    def test_duration_needs_both_phases(self):
        for overview in ({'stance': -1}, {'swing': -1}, {'stance': 0, 'swing': 0}):
            row = row_for(**overview)
            self.assertEqual((row.duration_s, row.cadence_hz, row.speed_mps), (None, None, None), overview)
        row = row_for(distance=-1)
        self.assertIsNone(row.distance_m)
        self.assertEqual(row.duration_s, 0.75)

    def test_csv_row(self):
        self.assertEqual(len(GAIT_CSV_HEADER.split(',')), 21)
        self.assertEqual(
            gait_row_to_csv(row_for()),
            '5,walk,forward,12.5000,0.5000,0.2500,0.7500,1.3333,6.6667,5.5000,3,4,0,5,1.2500,-5,heelStrike,'
            '-9.4000,neutral,2,1.5000')
        self.assertEqual(
            gait_row_to_csv(row_for(stance=-1)),
            '5,walk,forward,12.5000,,0.2500,,,,5.5000,3,4,0,5,1.2500,-5,heelStrike,-9.4000,neutral,2,1.5000')


async def connected(**kwargs):
    clients = []

    def factory(ble_device):
        client = FakeClient(ble_device, INSOLE_DEVICE_INFORMATION)
        clients.append(client)
        return client

    device = OrpheCoreInsole(insole_profile(), verbose=False, discover=fake_discover(("INS-L", "AA")),
                             client_factory=factory, monitor_interval_sec=0.01, **kwargs)
    assert await device.connect()
    return device, clients[0]


def notify(client, data):
    return client.notify[CHARACTERISTIC_STEP_ANALYSIS_UUID](None, bytearray(data))


class TestInsoleGait(unittest.IsolatedAsyncioTestCase):

    async def test_start_subscribes_and_emits_rows(self):
        device, client = await connected()
        gait = InsoleGait(device)
        rows, motions = [], []
        gait.on('gait', rows.append)
        gait.on('motion', motions.append)
        device_gait = []
        device.on('gait', device_gait.append)

        self.assertTrue(await gait.start())
        self.assertTrue(gait.is_running)
        for data in (overview_packet(1), motion_packet(1), stride_packet(1), pronation_packet(1)):
            await notify(client, data)

        self.assertEqual([row.step_number for row in rows], [1])
        self.assertEqual(len(motions), 1)
        self.assertEqual(gait.step_count, 1)
        self.assertIn('\n1,walk,forward,', gait.to_csv())
        self.assertEqual(device_gait, [])  # 横取り中は OrpheCoreInsole のイベントを出さない

        diagnostics = gait.diagnostics()
        self.assertTrue(diagnostics.subscribed)
        self.assertEqual((diagnostics.transport_notifications, diagnostics.valid_packets), (4, 4))
        self.assertEqual(diagnostics.step_loss.completed_steps, 1)

        await gait.stop()
        self.assertFalse(gait.is_running)
        self.assertNotIn(CHARACTERISTIC_STEP_ANALYSIS_UUID, client.notify)

    async def test_invalid_packet_is_counted_and_reported_once(self):
        device, client = await connected()
        gait = InsoleGait(device)
        diagnostics = []
        gait.on('diagnostic', lambda info: diagnostics.append(info['type']))
        await gait.start()
        await notify(client, bytes(5))
        await notify(client, bytes(5))
        self.assertEqual(gait.diagnostics().invalid_packets, 2)
        self.assertEqual(diagnostics, ['subscription-started', 'invalid-packet'])
        await gait.stop()

    async def test_cannot_start_when_not_connected(self):
        device = OrpheCoreInsole(insole_profile(), verbose=False)
        gait = InsoleGait(device)
        errors = []
        gait.on('error', errors.append)
        self.assertFalse(await gait.start())
        self.assertIn('not connected', str(errors[0]))

    async def test_only_one_gait_per_device(self):
        device, _ = await connected()
        first, second = InsoleGait(device), InsoleGait(device)
        errors = []
        second.on('error', errors.append)
        self.assertTrue(await first.start())
        self.assertFalse(await second.start())
        self.assertIn('already taken', str(errors[0]))

        await first.stop()
        self.assertTrue(await second.start())
        await second.stop()

    async def test_refresh_subscription_keeps_rows(self):
        device, client = await connected()
        gait = InsoleGait(device)
        await gait.start()
        for data in step_packets(1):
            await notify(client, data)
        self.assertTrue(await gait.refresh_subscription())
        self.assertEqual(len(gait.rows), 1)
        for data in step_packets(2):
            await notify(client, data)
        self.assertEqual(len(gait.rows), 2)
        await gait.stop()

    async def test_wait_for_packet(self):
        device, client = await connected()
        gait = InsoleGait(device)
        await gait.start()
        waiting = asyncio.ensure_future(gait.wait_for_packet(timeout_sec=1.0))
        await asyncio.sleep(0)
        await notify(client, overview_packet(1))
        self.assertTrue(await waiting)
        self.assertFalse(await gait.wait_for_packet(timeout_sec=0.01))
        await gait.stop()

    async def test_restart_resets_rows(self):
        device, client = await connected()
        gait = InsoleGait(device)
        await gait.start()
        for data in step_packets(1):
            await notify(client, data)
        await gait.stop()

        await gait.start()
        self.assertEqual(gait.rows, [])
        self.assertEqual(gait.diagnostics().step_loss.completed_steps, 0)
        await gait.stop()

    async def test_reset(self):
        device, client = await connected()
        gait = InsoleGait(device)
        await gait.start()
        for data in step_packets(1):
            await notify(client, data)
        gait.reset()  # 解析中は何もしない
        self.assertEqual(gait.step_count, 1)
        await gait.stop()

        gait.reset()
        self.assertEqual(gait.step_count, 0)
        diagnostics = gait.diagnostics()
        self.assertEqual((diagnostics.transport_notifications, diagnostics.valid_packets), (0, 0))
        self.assertIsNone(diagnostics.last_transport)
        self.assertEqual(diagnostics.step_loss.completed_steps, 0)

    async def test_disconnect_marks_unsubscribed(self):
        device, client = await connected()
        gait = InsoleGait(device)
        diagnostics = []
        gait.on('diagnostic', lambda info: diagnostics.append(info['type']))
        await gait.start()
        await client.disconnect()
        await asyncio.sleep(0.05)
        self.assertFalse(gait.diagnostics().subscribed)
        self.assertIn('physical-disconnect', diagnostics)
        await gait.stop()

    async def test_device_step_analysis_events_return_after_stop(self):
        device, client = await connected()
        gait = InsoleGait(device)
        await gait.start()
        await gait.stop()
        received = []
        device.on('gait', received.append)
        await device.start_step_analysis_notification()
        await notify(client, overview_packet(1))
        self.assertEqual(len(received), 1)
