"""
ORPHE CORE のプロファイル

換算・レイアウトは ORPHE-CORE.py（公開版）と同じ。
"""

from __future__ import annotations

from ..data import AccData, GyroData, QuatData, Range, to_timestamp
from ..packets import SensorFrame, SensorValuesPacket, stamp
from .base import DeviceKind, i16, imu_range_from_settings, u16

WRITE_WAIT_INTERVAL_SEC = 0.2

# SENSOR_VALUES のヘッダ
HEADER_V2 = 40  # 50Hz モデル（1 サンプル、シリアル番号なし）
HEADER_V3 = 50  # 200Hz モデル（4 サンプル）

# device information に書き込むコマンド
COMMAND_DEVICE_INFORMATION = 0x01
COMMAND_LED = 0x02
DEVICE_INFORMATION_WRITE_SIZE = 20


class CoreDeviceInformation:
    """
    ORPHE CORE のデバイス情報

    Attributes:
        data: 生データ
        battery: バッテリー残量
        lr: 左右
        rec: REC
        auto_run: Auto Run
        led: LED 発光の強さ 0-255
        log_high: Log High
        log_low: Log Low
        range: レンジ
    """

    def __init__(self, data: bytes | bytearray) -> None:
        self.data = data
        self.battery = data[0]
        self.lr = data[1]
        self.rec = data[2]
        self.auto_run = data[3]
        self.led = data[4]
        self.log_high = data[6]
        self.log_low = data[7]
        self.range = Range()
        self.range.acc = data[8]
        self.range.gyro = data[9]
        self.device_information = None


def build_device_information_write(di: CoreDeviceInformation, *, lr: int | None = None,
                                   led: int | None = None) -> bytearray:
    """device information の書き込みバイト列（lr / led を省略すると di の値を使う）"""
    return bytearray([
        COMMAND_DEVICE_INFORMATION,
        di.lr if lr is None else lr,
        di.led if led is None else led,
        0x00,
        di.auto_run,
        di.log_high,
        di.log_low,
        di.range.acc,
        di.range.gyro,
    ] + [0x00] * 11)


def build_led_write(is_on: int, pattern: int) -> bytearray:
    """
    LED の点灯・消灯

    Args:
        is_on: 0 or 1
        pattern: 0-4
    """
    if is_on < 0 or is_on > 1:
        raise ValueError("is_on must be 0 or 1.")
    if pattern < 0 or pattern > 4:
        raise ValueError("pattern must be 0-4.")
    return bytearray([COMMAND_LED, is_on, pattern] + [0x00] * 17)


def validate_led_brightness(brightness: int) -> None:
    if brightness < 0 or brightness > 255:
        raise ValueError("brightness must be 0-255.")


def validate_lr(lr: int) -> None:
    if lr < 0 or lr > 1:
        raise ValueError("lr must be 0 or 1.")


def _parse_v3(data: bytes | bytearray, sensor_range: Range) -> SensorValuesPacket:
    """
    ヘッダ 50（200Hz、4 サンプル）

    サンプルは 21 バイト間隔: quat(8) gyro(6) acc(6) 経過ms(1)。
    パケット末尾のサンプルから packet_number 0 として通知する。
    """
    serial_number = u16(data, 1)
    timestamp = to_timestamp(data[3], data[4], data[5], data[6], data[7])
    packet = SensorValuesPacket(HEADER_V3, serial_number, timestamp)
    amp_acc, amp_gyro = imu_range_from_settings(sensor_range)
    each_timestamp = timestamp

    for i in range(3, -1, -1):
        step = 21 * i
        acc = AccData()
        acc.x = i16(data, 22 + step) / 32768
        acc.y = i16(data, 24 + step) / 32768
        acc.z = i16(data, 26 + step) / 32768

        converted_acc = AccData()
        converted_acc.x = acc.x * amp_acc
        converted_acc.y = acc.y * amp_acc
        converted_acc.z = acc.z * amp_acc

        gyro = GyroData()
        gyro.x = i16(data, 16 + step) / 32768
        gyro.y = i16(data, 18 + step) / 32768
        gyro.z = i16(data, 20 + step) / 32768

        converted_gyro = GyroData()
        converted_gyro.x = gyro.x * amp_gyro
        converted_gyro.y = gyro.y * amp_gyro
        converted_gyro.z = gyro.z * amp_gyro

        # CORE のクォータニオンは Q15
        quat = QuatData()
        quat.w = i16(data, 8 + step) / 32768
        quat.x = i16(data, 10 + step) / 32768
        quat.y = i16(data, 12 + step) / 32768
        quat.z = i16(data, 14 + step) / 32768

        if i != 3:
            each_timestamp = each_timestamp + data[28 + step]
        stamp([acc, converted_acc, gyro, converted_gyro, quat], each_timestamp, serial_number, 3 - i)
        packet.frames.append(SensorFrame(acc, converted_acc, gyro, converted_gyro, quat=quat))
    return packet


def _parse_v2(data: bytes | bytearray, sensor_range: Range) -> SensorValuesPacket:
    """ヘッダ 40（50Hz モデル、1 サンプル、8 ビット精度）"""
    timestamp = u16(data, 18)
    packet = SensorValuesPacket(HEADER_V2, None, timestamp)

    acc = AccData()
    acc.x = int.from_bytes(data[14:15], byteorder='big', signed=True) / 127
    acc.y = int.from_bytes(data[15:16], byteorder='big', signed=True) / 127
    acc.z = int.from_bytes(data[16:17], byteorder='big', signed=True) / 127

    amp_acc, amp_gyro = imu_range_from_settings(sensor_range)
    converted_acc = AccData()
    converted_acc.x = acc.x * amp_acc
    converted_acc.y = acc.y * amp_acc
    converted_acc.z = acc.z * amp_acc

    gyro = GyroData()
    gyro.x = int.from_bytes(data[9:10], byteorder='big', signed=True) / 127
    gyro.y = int.from_bytes(data[10:11], byteorder='big', signed=True) / 127
    gyro.z = int.from_bytes(data[11:12], byteorder='big', signed=True) / 127

    converted_gyro = GyroData()
    converted_gyro.x = gyro.x * amp_gyro
    converted_gyro.y = gyro.y * amp_gyro
    converted_gyro.z = gyro.z * amp_gyro

    quat = QuatData()
    quat.w = i16(data, 1) / 32768
    quat.x = i16(data, 3) / 32768
    quat.y = i16(data, 5) / 32768
    quat.z = i16(data, 7) / 32768

    # 50Hz モデルの FW はシリアル番号・パケット番号を送らない
    stamp([acc, converted_acc, gyro, converted_gyro, quat], timestamp, 0, 0)
    packet.frames.append(SensorFrame(acc, converted_acc, gyro, converted_gyro, quat=quat))
    return packet


class CoreProfile:
    """ORPHE CORE"""

    kind: DeviceKind = 'core'
    label = 'ORPHE CORE'
    write_wait_sec = WRITE_WAIT_INTERVAL_SEC
    lost_data_headers = frozenset({HEADER_V3})

    def matches_name(self, name: str | None) -> bool:
        return name is not None and "CR-" in name

    def parse_device_information(self, data: bytes | bytearray) -> CoreDeviceInformation:
        return CoreDeviceInformation(data)

    def describe_device_information(self, di: CoreDeviceInformation) -> list[str]:  # type: ignore[override]
        return [
            f"Battery: {di.battery}",
            f"LR: {di.lr}",
            f"REC: {di.rec}",
            f"Auto Run: {di.auto_run}",
            f"LED: {di.led}",
            f"Log High: {di.log_high}",
            f"Log Low: {di.log_low}",
            f"ACC Range: {di.range.acc}",
            f"GYRO Range: {di.range.gyro}",
        ]

    def build_range_write(self, di: CoreDeviceInformation) -> bytearray:  # type: ignore[override]
        return build_device_information_write(di)

    def parse_sensor_values(self, data: bytes | bytearray, sensor_range: Range) -> SensorValuesPacket | None:
        if data[0] == HEADER_V3:
            return _parse_v3(data, sensor_range)
        if data[0] == HEADER_V2:
            return _parse_v2(data, sensor_range)
        return None


def core_profile() -> CoreProfile:
    return CoreProfile()
