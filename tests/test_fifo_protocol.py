"""FIFO のプロトコル・ループ状態（BLE なし）"""

import time
import unittest

from fifo_sim import build_fifo_packet, expected_raw, firmware_name

from orphe_core_insole.fifo.protocol import (
    CORE_LAYOUTS,
    FIFO_CSV_HEADER_IMU,
    FIFO_CSV_HEADER_PRESSURE,
    FIFO_FRAME_INTERVAL_MS,
    INSOLE_LAYOUTS,
    FifoScale,
    _to_fixed,
    build_requests_from_serials,
    calc_expected_serials,
    create_get_sensor_data_request,
    decode_fifo_packet,
    expand_requests_to_list,
    extract_timestamp_ms,
    layout_of,
    packet_to_csv_rows,
    parse_current_serial,
    parse_no_data_response,
    polynomial_pressure_to_newton,
    raw_store_to_csv,
    serial_distance,
    timestamp_to_str,
)
from orphe_core_insole.fifo.state import DrainBudget, FifoLoopState, FifoLossEvent
from orphe_core_insole.firmware import decode_firmware_info, supports_fifo


class TestConversion(unittest.TestCase):

    def test_scale(self):
        full = FifoScale()
        self.assertEqual(full.acc(2048), 1.0)
        self.assertEqual(full.acc(-32768), -16.0)
        self.assertEqual(full.gyro(100), 6.999999999999999)
        self.assertEqual(FifoScale(2, 250).acc(16384), 1.0)
        self.assertEqual(FifoScale(2, 250).gyro(1000), 8.75)

    def test_pressure_to_newton(self):
        self.assertAlmostEqual(polynomial_pressure_to_newton(0, 1), 22.5012)
        self.assertEqual(polynomial_pressure_to_newton(1000, 7), 0)
        self.assertEqual(polynomial_pressure_to_newton(0, 6), 0)  # 多項式が負（-4.4658）なら 0

    def test_to_fixed_matches_javascript(self):
        self.assertEqual(_to_fixed(0.125, 2), '0.13')    # 0.5 は切り上げ（Python の丸めは 0.12）
        self.assertEqual(_to_fixed(-0.125, 2), '-0.13')
        self.assertEqual(_to_fixed(-0.0, 2), '0.00')
        self.assertEqual(_to_fixed(-0.001, 2), '-0.00')
        self.assertEqual(_to_fixed(1.005, 2), '1.00')    # 1.005 は 2 進数で 1.00499…
        self.assertEqual(_to_fixed(3.5e59, 4), '3.5e+59')  # 1e21 以上は指数表記
        self.assertEqual(_to_fixed(123456789012.34567, 4), '123456789012.3457')


class TestSerials(unittest.TestCase):

    def test_distance_and_expected(self):
        self.assertEqual(serial_distance(65534, 2), 4)
        self.assertEqual(serial_distance(10, 10), 0)
        self.assertEqual(calc_expected_serials(65534, 4), [65534, 65535, 0, 1])

    def test_requests(self):
        self.assertEqual(build_requests_from_serials({42, 10, 12, 40, 41}), [(10, 1), (12, 1), (40, 3)])
        self.assertEqual(build_requests_from_serials({65535, 0, 1}), [(0, 2), (65535, 1)])
        self.assertEqual(build_requests_from_serials([]), [])
        self.assertEqual(expand_requests_to_list([(65535, 2), (5, 1)]), [65535, 0, 5])

    def test_get_sensor_data_request(self):
        request = create_get_sensor_data_request([(0x1234, 3), (1, 200)])
        self.assertEqual(len(request), 122)
        self.assertEqual(request[:10], bytes([0x0B, 0x02, 0x12, 0x34, 0, 3, 0, 1, 0, 200]))
        self.assertEqual(request[10:], bytes(112))
        with self.assertRaises(ValueError):
            create_get_sensor_data_request([(i, 1) for i in range(31)])

    def test_compact_request(self):
        from orphe_core_insole.fifo.protocol import max_pairs_per_request
        self.assertEqual(max_pairs_per_request(122), 30)
        self.assertEqual(max_pairs_per_request(104), 24)  # (104 - 2) // 4 = 25 組、うち 1 組は終端
        request = create_get_sensor_data_request([(1, 2), (5, 1)], max_size=104)
        self.assertEqual(request, bytes([0x0B, 0x02, 0, 1, 0, 2, 0, 5, 0, 1, 0, 0, 0, 0]))
        with self.assertRaises(ValueError):
            create_get_sensor_data_request([(i, 1) for i in range(25)], max_size=104)

    def test_responses(self):
        self.assertEqual(parse_no_data_response(bytes([0x35, 0x02, 0, 10, 0, 3])), (10, 3))
        self.assertEqual(parse_no_data_response(bytes([0x35, 0x02])), (0, 0))
        self.assertIsNone(parse_no_data_response(bytes([0x35, 0x01, 0, 0])))
        current = parse_current_serial(bytes([0x35, 0x01, 0x01, 0x00, 1, 0x02, 0x00]))
        self.assertEqual((current.serial, current.watermark, current.accumulated), (256, 1, 512))
        self.assertIsNone(parse_current_serial(bytes([0x35, 0x01, 0, 0])))


class TestTimestamp(unittest.TestCase):

    def test_to_str(self):
        self.assertEqual(timestamp_to_str(12, 30, 15, 998, 5), '12:30:16:003')
        self.assertEqual(timestamp_to_str(23, 59, 59, 999, 1), '00:00:00:000')

    def test_extract_ms(self):
        packet = build_fifo_packet('insole', 0x36, 1, (1 * 3600 + 2 * 60 + 3) * 1000 + 45)
        self.assertEqual(extract_timestamp_ms(packet), 3_723_045)


class TestDecode(unittest.TestCase):

    def check_layout(self, kind, header, samples):
        packet = build_fifo_packet(kind, header, 500, 0)
        layout = layout_of(packet, kind)
        self.assertIsNotNone(layout)
        self.assertEqual(layout.samples, samples)
        decoded = decode_fifo_packet(packet, layout, FifoScale())
        self.assertEqual(len(decoded), samples)
        for packet_number, sample in enumerate(decoded):
            # 古い順 = バイト列の末尾（index samples-1）から
            gyro, _acc, pressure = expected_raw(500, samples - 1 - packet_number)
            self.assertEqual(sample.packet_number, packet_number)
            self.assertEqual(sample.gyro[2], gyro[2] * 2000 * 0.000035)
            self.assertEqual(sample.acc, (0.0, 0.0, 1.0))
            self.assertEqual(sample.pressure, pressure if kind == 'insole' else None)
            self.assertAlmostEqual(sample.t, packet_number * FIFO_FRAME_INTERVAL_MS)

    def test_insole_4(self):
        self.check_layout('insole', 0x36, 4)

    def test_core_8x12(self):
        self.check_layout('core', 0x36, 8)

    def test_layout_checks_kind_and_length(self):
        core_packet = build_fifo_packet('core', 0x36, 1, 0)
        insole_packet = build_fifo_packet('insole', 0x36, 1, 0)
        self.assertEqual(len(core_packet), len(insole_packet))  # どちらも 104 バイト
        self.assertIs(layout_of(core_packet, 'core'), CORE_LAYOUTS[0x36])
        self.assertIs(layout_of(insole_packet, 'insole'), INSOLE_LAYOUTS[0x36])
        self.assertIsNone(layout_of(bytes([0x37]) + core_packet[1:], 'core'))  # 知らないヘッダ
        self.assertIsNone(layout_of(insole_packet[:-1], 'insole'))
        self.assertIsNone(layout_of(bytes([0x35, 0x02]), 'insole'))


class TestCsv(unittest.TestCase):

    def test_insole_rows(self):
        packet = build_fifo_packet('insole', 0x36, 7, 12 * 3_600_000 + 998)
        rows = packet_to_csv_rows(packet, INSOLE_LAYOUTS[0x36], FifoScale())
        self.assertEqual(len(rows), 4)
        self.assertTrue(rows[0].startswith('7, 12:00:00:998,     0.49,    -0.49,     0.21,   0.0000'))
        self.assertTrue(rows[1].startswith('7, 12:00:01:003,'))
        self.assertEqual(len(rows[0].split(', ')), 14)

    def test_core_rows_have_no_pressure(self):
        rows = packet_to_csv_rows(build_fifo_packet('core', 0x36, 7, 0), CORE_LAYOUTS[0x36], FifoScale())
        self.assertEqual(len(rows), 8)
        self.assertEqual(len(rows[0].split(', ')), 8)

    def test_store_sorted_by_time(self):
        later = build_fifo_packet('insole', 0x36, 2, 2000)
        earlier = build_fifo_packet('insole', 0x36, 1, 1000)
        csv = raw_store_to_csv({2: later, 1: earlier, 3: bytes([0x35, 0x02])}, 'insole', FifoScale())
        lines = csv.splitlines()
        self.assertEqual(lines[0], FIFO_CSV_HEADER_PRESSURE)
        self.assertEqual([line.split(',')[0] for line in lines[1:]], ['1'] * 4 + ['2'] * 4)
        self.assertEqual(raw_store_to_csv({}, 'core', FifoScale()), FIFO_CSV_HEADER_IMU + '\n')


class TestLoopState(unittest.TestCase):

    def test_calc_request_range(self):
        state = FifoLoopState()
        self.assertEqual(state.calc_request_range(100, 30, 200), (71, 30))  # 未同期なら直近 accumulated 件
        state.last_serial = 100
        self.assertEqual(state.calc_request_range(110, 10, 200), (101, 10))
        self.assertEqual(state.calc_request_range(110, 10, 4), (101, 4))
        state.last_serial = 65534
        self.assertEqual(state.calc_request_range(3, 5, 200), (65535, 5))
        self.assertEqual(FifoLoopState().calc_request_range(42, 0, 200), (0, 0))
        self.assertEqual(state.dropped, 0)

    def test_ring_overflow(self):
        state = FifoLoopState()
        state.last_serial = 0
        state.carry_over = [(1, 1)]
        self.assertEqual(state.calc_request_range(1600, 1500, 200), (101, 200))
        self.assertEqual((state.last_serial, state.dropped, state.carry_over), (100, 100, []))
        self.assertEqual(state.loss_events, [FifoLossEvent('ring_overflow', 100)])

    def test_update_after_response(self):
        state = FifoLoopState()
        state.last_serial = 100
        self.assertEqual(state.update_after_response({103, 104, 107}, set(), 101, 10), 'ok')
        self.assertEqual(state.last_serial, 110)
        self.assertEqual(state.carry_over, [(103, 2), (107, 1)])

    def test_carry_over_overflow_then_resync_backlog(self):
        state = FifoLoopState()
        state.last_serial = 0
        self.assertEqual(state.update_after_response(set(range(1, 102)), set(), 1, 150), 'resync')
        self.assertEqual((state.dropped, state.last_serial, state.resync_pending), (101, None, True))
        self.assertEqual(state.calc_request_range(500, 300, 200), (301, 200))
        self.assertEqual(state.dropped, 201)
        self.assertEqual(state.loss_events, [FifoLossEvent('carryover_overflow', 101),
                                             FifoLossEvent('resync_backlog', 100)])

    def test_new_no_data_resyncs_but_keeps_carry_over(self):
        state = FifoLoopState()
        state.last_serial = 10
        state.carry_over = [(5, 1)]
        self.assertEqual(state.update_after_response(set(), {11}, 11, 3), 'ok')
        self.assertEqual((state.last_serial, state.resync_pending, state.carry_over), (None, True, [(5, 1)]))

    def test_span_and_pending_loss(self):
        state = FifoLoopState()
        state.note_stored(10)
        state.note_stored(12)
        self.assertEqual((state.first_stored_serial, state.stored_span_max), (10, 2))
        state.note_stored(9)
        self.assertEqual((state.first_stored_serial, state.stored_span_max), (9, 3))
        for serial in (9, 10, 12):
            state.raw_store[serial] = b''
        state.note_span_target(15)
        state.note_span_target(9 + 40000)  # 半周を超える値は無視
        self.assertEqual(state.stored_span_max, 6)
        self.assertEqual(state.finalize_pending_loss(), 4)
        self.assertEqual(state.loss_events, [FifoLossEvent('stopped_pending', 4)])
        self.assertEqual(state.finalize_pending_loss(), 0)

        wrapped = FifoLoopState()
        wrapped.note_stored(65535)
        wrapped.note_stored(1)
        self.assertEqual((wrapped.first_stored_serial, wrapped.stored_span_max), (65535, 2))

    def test_drain_budget(self):
        budget = DrainBudget(0.1, now=1000.0)
        self.assertEqual((budget.budget_sec, budget.deadline, budget.hard_deadline), (0.1, 1000.1, 1001.0))
        negative = DrainBudget(-5, now=1000.0)
        self.assertEqual((negative.deadline, negative.hard_deadline), (1000.0, 1000.0))
        live = DrainBudget(0.5)
        self.assertTrue(0 < live.remaining_sec() <= 0.5)
        self.assertFalse(live.expired)
        self.assertTrue(DrainBudget(0.0, now=time.monotonic()).expired)


class TestFirmware(unittest.TestCase):

    def test_decode(self):
        info = decode_firmware_info(firmware_name(20260905, "TEST-BUILD"))
        self.assertEqual((info.build_id, info.major, info.minor, info.patch), ("TEST-BUILD", 1, 3, 7))
        self.assertEqual(info.release_date, 20260905)
        self.assertEqual(info.name, "TEST-BUILD-1-0003-20260905-07")
        self.assertIsNone(decode_firmware_info(bytes(10)))
        self.assertIsNone(decode_firmware_info(bytes(23)))  # 日付が 0

    def test_supports_fifo(self):
        self.assertTrue(supports_fifo('core', decode_firmware_info(firmware_name(20260905))))
        self.assertFalse(supports_fifo('core', decode_firmware_info(firmware_name(20260904))))
        self.assertTrue(supports_fifo('insole', decode_firmware_info(firmware_name(20260510))))
        self.assertIsNone(supports_fifo('insole', None))


if __name__ == "__main__":
    unittest.main()
