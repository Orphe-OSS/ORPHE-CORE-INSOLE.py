"""CORE / INSOLE プロファイルのパースとコマンドのバイト列"""

import unittest

from helpers import (
    CORE_DEVICE_INFORMATION,
    INSOLE_DEVICE_INFORMATION,
    build_packet_40,
    build_packet_50,
    build_packet_55,
    build_packet_56,
    i16,
)

from orphe_core_insole.data import Range
from orphe_core_insole.fifo.protocol import FifoScale
from orphe_core_insole.profiles import AutoProfile, CoreProfile, InsoleProfile, detect_device_kind
from orphe_core_insole.profiles.base import acc_range_index, gyro_range_index, gyro_raw_to_dps, imu_range_from_settings
from orphe_core_insole.profiles.core import CoreDeviceInformation, build_led_write
from orphe_core_insole.profiles.core import build_device_information_write as core_write
from orphe_core_insole.profiles.insole import (
    InsoleDeviceInformation,
    build_streaming_mode_write,
    checksum,
)
from orphe_core_insole.profiles.insole import build_device_information_write as insole_write


def sensor_range(acc: int, gyro: int) -> Range:
    r = Range()
    r.acc = acc
    r.gyro = gyro
    return r


class TestImuRange(unittest.TestCase):

    def test_imu_range_from_settings(self):
        # インデックスを物理値にする。未取得・範囲外・非整数は 16G / 2000dps
        self.assertEqual(imu_range_from_settings(sensor_range(0, 3)), (2, 2000))
        self.assertEqual(imu_range_from_settings(sensor_range(2, 1)), (8, 500))
        self.assertEqual(imu_range_from_settings(None), (16, 2000))
        self.assertEqual(imu_range_from_settings(sensor_range(4, -1)), (16, 2000))
        self.assertEqual(imu_range_from_settings(sensor_range(1.5, '1')), (16, 2000))  # type: ignore[arg-type]

    def test_gyro_raw_to_dps(self):
        # ±2000dps で 70 mdps/LSB、感度はレンジに比例する
        self.assertEqual(gyro_raw_to_dps(1000, 2000), 70)
        self.assertEqual(gyro_raw_to_dps(1000, 250), 8.75)
        self.assertEqual(gyro_raw_to_dps(-1000, 500), -17.5)

    def test_realtime_and_fifo_use_same_range(self):
        # 同じレンジ設定なら、INSOLE のリアルタイム計測と FIFO で同じ角速度・加速度になる
        for acc in range(4):
            for gyro in range(4):
                with self.subTest(acc=acc, gyro=gyro):
                    data = build_packet_55(gyro=(1000, 0, 0), acc=(8192, 0, 0), press=(0,) * 6)
                    frame = InsoleProfile().parse_sensor_values(data, sensor_range(acc, gyro)).frames[0]
                    scale = FifoScale(*imu_range_from_settings(sensor_range(acc, gyro)))
                    self.assertAlmostEqual(frame.converted_gyro.x, scale.gyro(1000))
                    self.assertAlmostEqual(frame.converted_acc.x, scale.acc(8192))

    def test_out_of_range_setting_falls_back(self):
        # 範囲外のインデックスは 16G / 2000dps で換算する（IndexError にしない）
        data = build_packet_50(quat=(0, 0, 0, 0), gyro=(16384, 0, 0), acc=(8192, 0, 0))
        frame = CoreProfile().parse_sensor_values(data, sensor_range(4, 7)).frames[0]
        self.assertEqual((frame.converted_acc.x, frame.converted_gyro.x), (4.0, 1000.0))
        frame = InsoleProfile().parse_sensor_values(data, sensor_range(4, 7)).frames[0]
        self.assertEqual(frame.converted_acc.x, 4.0)
        self.assertAlmostEqual(frame.converted_gyro.x, 16384 * 0.07)


class TestCoreSensorValues(unittest.TestCase):

    def test_header_50_scales_with_range(self):
        data = build_packet_50(quat=(16384, 0, 0, -16384), gyro=(16384, 0, -8192),
                               acc=(8192, -4096, 2048), serial=7)
        packet = CoreProfile().parse_sensor_values(data, sensor_range(acc=3, gyro=3))
        self.assertEqual(packet.header, 50)
        self.assertEqual(packet.serial_number, 7)
        self.assertEqual(len(packet.frames), 4)
        frame = packet.frames[0]
        self.assertEqual((frame.acc.x, frame.acc.y, frame.acc.z), (0.25, -0.125, 0.0625))
        # 加速度は ±16G、ジャイロは ±2000dps（CORE は raw/32768 × レンジ）
        self.assertEqual(frame.converted_acc.x, 4.0)
        self.assertEqual(frame.converted_gyro.x, 1000.0)
        self.assertEqual(frame.converted_gyro.z, -500.0)
        # CORE のクォータニオンは Q15
        self.assertEqual((frame.quat.w, frame.quat.z), (0.5, -0.5))
        self.assertIsNone(frame.pressure)

    def test_header_50_frame_order_and_timestamps(self):
        # 末尾のサンプルから packet_number 0 として並ぶ。経過 ms を足していく
        data = bytearray(build_packet_50(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0)))
        for i in range(4):
            data[22 + 21 * i:24 + 21 * i] = i16(100 * (i + 1))  # acc.x でサンプルを識別
        packet = CoreProfile().parse_sensor_values(bytes(data), sensor_range(0, 0))
        self.assertEqual([round(f.acc.x * 32768) for f in packet.frames], [400, 300, 200, 100])
        self.assertEqual([f.acc.packet_number for f in packet.frames], [0, 1, 2, 3])
        base = packet.timestamp
        self.assertEqual([f.quat.timestamp - base for f in packet.frames], [0, 5, 10, 15])
        self.assertTrue(all(f.gyro.serial_number == 1 for f in packet.frames))

    def test_header_40(self):
        data = build_packet_40(quat=(32767, 0, 0, 0), gyro=(127, -127, 0), acc=(0, 0, 127), timestamp=999)
        packet = CoreProfile().parse_sensor_values(data, sensor_range(acc=1, gyro=0))
        self.assertIsNone(packet.serial_number)
        frame, = packet.frames
        self.assertEqual(frame.acc.z, 1.0)
        self.assertEqual(frame.converted_acc.z, 4.0)
        self.assertEqual((frame.converted_gyro.x, frame.converted_gyro.y), (250.0, -250.0))
        self.assertEqual(frame.quat.timestamp, 999)
        self.assertEqual((frame.acc.serial_number, frame.acc.packet_number), (0, 0))

    def test_ignores_insole_headers(self):
        data = build_packet_56(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0), press=(0,) * 6)
        self.assertIsNone(CoreProfile().parse_sensor_values(data, sensor_range(0, 0)))


class TestInsoleSensorValues(unittest.TestCase):

    def test_header_50_uses_q14_and_gyro_sensitivity(self):
        data = build_packet_50(quat=(16384, 0, 0, 0), gyro=(1000, 0, 0), acc=(0, 0, 0))
        packet = InsoleProfile().parse_sensor_values(data, sensor_range(0, 3))
        self.assertEqual(packet.frames[0].quat.w, 1.0)
        self.assertAlmostEqual(packet.frames[0].converted_gyro.x, 1000 * 0.07)

    def test_header_55_has_pressure_without_quat(self):
        press = (1, 2, 3, 4, 5, 65535)
        data = build_packet_55(gyro=(0, 0, 0), acc=(0, 0, 0), press=press, serial=300)
        packet = InsoleProfile().parse_sensor_values(data, sensor_range(0, 0))
        self.assertEqual(packet.serial_number, 300)
        self.assertEqual(len(packet.frames), 4)
        for frame in packet.frames:
            self.assertEqual(frame.pressure.values, list(press))
            self.assertIsNone(frame.quat)

    def test_header_56_two_frames_share_timestamp(self):
        data = build_packet_56(quat=(16384, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0), press=(9,) * 6)
        packet = InsoleProfile().parse_sensor_values(data, sensor_range(0, 0))
        self.assertEqual([f.pressure.packet_number for f in packet.frames], [0, 1])
        self.assertEqual(len({f.quat.timestamp for f in packet.frames}), 1)

    def test_ignores_core_v2_header(self):
        data = build_packet_40(quat=(0, 0, 0, 0), gyro=(0, 0, 0), acc=(0, 0, 0))
        self.assertIsNone(InsoleProfile().parse_sensor_values(data, sensor_range(0, 0)))


class TestCommands(unittest.TestCase):

    def test_core_device_information_write(self):
        di = CoreDeviceInformation(CORE_DEVICE_INFORMATION)
        self.assertEqual(list(core_write(di)), [1, 1, 200, 0, 1, 3, 4, 2, 3] + [0] * 11)
        self.assertEqual(list(core_write(di, led=10, lr=0))[:3], [1, 0, 10])

    def test_core_led(self):
        self.assertEqual(list(build_led_write(1, 4)), [2, 1, 4] + [0] * 17)
        with self.assertRaisesRegex(ValueError, "is_on must be 0 or 1."):
            build_led_write(2, 0)
        with self.assertRaisesRegex(ValueError, "pattern must be 0-4."):
            build_led_write(1, 5)

    def test_insole_device_information_write_has_checksum(self):
        di = InsoleDeviceInformation(INSOLE_DEVICE_INFORMATION)
        data = insole_write(di)
        self.assertEqual(len(data), 20)
        self.assertEqual(list(data[:19]),
                         [9, 1, 0, 0, 1, 0, 0x3C, 3, 3, 0, 0, 0, 0xFF, 0, 0, 0, 0, 0, 5])
        self.assertEqual(data[19], (9 + 1 + 1 + 0x3C + 3 + 3 + 0xFF + 5) & 0xFF)
        self.assertEqual(data[19], checksum(data))

    def test_insole_streaming_mode(self):
        self.assertEqual(build_streaming_mode_write(3), bytearray([0x0D, 3]))
        with self.assertRaisesRegex(ValueError, "mode must be 1, 3, or 4."):
            build_streaming_mode_write(2)

    def test_range_index(self):
        self.assertEqual(acc_range_index(16), 3)
        self.assertEqual(gyro_range_index(250), 0)
        with self.assertRaisesRegex(ValueError, r"acc_range must be 2, 4, 8, or 16\[g\]."):
            acc_range_index(3)
        with self.assertRaises(ValueError):
            gyro_range_index(3)


class TestDeviceInformation(unittest.TestCase):

    def test_core(self):
        di = CoreDeviceInformation(CORE_DEVICE_INFORMATION)
        self.assertEqual((di.battery, di.lr, di.led, di.log_high, di.log_low), (80, 1, 200, 3, 4))
        self.assertEqual((di.range.acc, di.range.gyro), (2, 3))

    def test_insole(self):
        di = InsoleDeviceInformation(INSOLE_DEVICE_INFORMATION)
        self.assertEqual((di.battery, di.mount_position, di.version), (70, 1, 5))
        self.assertEqual((di.range.acc, di.range.gyro), (3, 3))


class TestDeviceKind(unittest.TestCase):

    def test_detect(self):
        self.assertEqual(detect_device_kind("INS-L-0001"), 'insole')
        self.assertEqual(detect_device_kind("CR-3"), 'core')
        self.assertEqual(detect_device_kind(None), 'core')

    def test_auto_matches_both(self):
        auto = AutoProfile()
        self.assertTrue(auto.matches_name("CR-3"))
        self.assertTrue(auto.matches_name("INS-R"))
        self.assertFalse(auto.matches_name("Keyboard"))
        self.assertFalse(auto.matches_name(None))
        self.assertIsInstance(auto.resolve("INS-R"), InsoleProfile)
        self.assertIsInstance(auto.resolve("CR-3"), CoreProfile)


if __name__ == "__main__":
    unittest.main()
