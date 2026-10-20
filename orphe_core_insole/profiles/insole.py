"""
ORPHE INSOLE のプロファイル

換算・レイアウトは ORPHE-INSOLE.py（公開版）と同じ。
"""

from __future__ import annotations

from ..data import AccData, GyroData, PressureData, QuatData, Range, to_timestamp
from ..packets import SensorFrame, SensorValuesPacket, stamp
from .base import GYRO_DPS_PER_LSB_PER_RANGE, DeviceKind, i16, imu_range_from_settings, u16

WRITE_WAIT_INTERVAL_SEC = 0.5

# 正規化値（raw / 32768）から物理値[dps]へ換算する補正係数（32768 * 0.000035 = 1.14688）。
# 正規化値（gyro）は後方互換のため raw / 32768 のまま。
GYRO_SENSITIVITY_SCALE = 32768 * GYRO_DPS_PER_LSB_PER_RANGE

# INSOLE の FW はクォータニオンを符号付き Q14（1.0 = 16384）で送る
QUAT_SCALE = 16384

# SENSOR_VALUES のヘッダ
HEADER_QUAT_GYRO_ACC = 50        # streaming mode 1: quat + gyro + acc（200Hz、4 サンプル）
HEADER_GYRO_ACC_PRESS = 55       # streaming mode 3: gyro + acc + 圧力（200Hz、4 サンプル）
HEADER_QUAT_GYRO_ACC_PRESS = 56  # streaming mode 4: quat + gyro + acc + 圧力（100Hz、2 サンプル）

# device information に書き込むコマンド
COMMAND_DEVICE_INFORMATION = 0x09
COMMAND_STREAMING_MODE = 0x0D
STREAMING_MODES = (1, 3, 4)


class InsoleDeviceInformation:
    """
    ORPHE INSOLE のデバイス情報

    Attributes:
        data: 生データ
        battery: バッテリー残量
        mount_position: マウント位置（bit0: 左右、bit1: 0=足底 / 1=足背）
        range: レンジ
        version: ファームウェアバージョン
    """

    def __init__(self, data: bytes | bytearray) -> None:
        self.data = data
        self.battery = data[0]
        self.mount_position = data[1]
        self.range = Range()
        self.range.acc = data[8]
        self.range.gyro = data[9]
        self.version = data[18]
        self.device_information = None


def checksum(data: bytes | bytearray) -> int:
    """device information の先頭 19 バイトの和の下位 8 ビット"""
    return sum(data[0:19]) & 0xFF


def build_device_information_write(di: InsoleDeviceInformation) -> bytearray:
    """di の内容を書き込むバイト列（19 バイト + チェックサム）"""
    data = bytearray([
        COMMAND_DEVICE_INFORMATION,
        di.mount_position,
        0x00,
        0x00,
        0x01,
        0x00,
        0x3C,
        di.range.acc,
        di.range.gyro,
        0x00,
        0x00,
        0x00,
        0xFF,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        di.version,
    ])
    return data + bytearray([checksum(data)])


def build_streaming_mode_write(mode: int) -> bytearray:
    """
    データストリーミングモードの切り替え

    Args:
        mode: 1（quat・gyro・acc 200Hz）、3（gyro・acc・圧力 200Hz）、4（quat・gyro・acc・圧力 100Hz）
    """
    if mode not in STREAMING_MODES:
        raise ValueError("mode must be 1, 3, or 4.")
    return bytearray([COMMAND_STREAMING_MODE, mode])


def validate_mount_position(mount_position: int) -> None:
    if mount_position < 0 or mount_position > 3:
        raise ValueError("mount_position must be 0, 1, 2, 3.")


def _imu(data: bytes | bytearray, gyro_at: int, acc_at: int,
         sensor_range: Range) -> tuple[AccData, AccData, GyroData, GyroData]:
    """gyro(6) と acc(6) を読み、レンジで換算する"""
    acc = AccData()
    acc.x = i16(data, acc_at) / 32768
    acc.y = i16(data, acc_at + 2) / 32768
    acc.z = i16(data, acc_at + 4) / 32768

    amp_acc, gyro_range = imu_range_from_settings(sensor_range)
    converted_acc = AccData()
    converted_acc.x = acc.x * amp_acc
    converted_acc.y = acc.y * amp_acc
    converted_acc.z = acc.z * amp_acc

    gyro = GyroData()
    gyro.x = i16(data, gyro_at) / 32768
    gyro.y = i16(data, gyro_at + 2) / 32768
    gyro.z = i16(data, gyro_at + 4) / 32768

    converted_gyro = GyroData()
    amp_gyro = gyro_range * GYRO_SENSITIVITY_SCALE
    converted_gyro.x = gyro.x * amp_gyro
    converted_gyro.y = gyro.y * amp_gyro
    converted_gyro.z = gyro.z * amp_gyro
    return acc, converted_acc, gyro, converted_gyro


def _quat(data: bytes | bytearray, at: int) -> QuatData:
    quat = QuatData()
    quat.w = i16(data, at) / QUAT_SCALE
    quat.x = i16(data, at + 2) / QUAT_SCALE
    quat.y = i16(data, at + 4) / QUAT_SCALE
    quat.z = i16(data, at + 6) / QUAT_SCALE
    return quat


def _pressure(data: bytes | bytearray, at: int) -> PressureData:
    pressure = PressureData()
    pressure.values = [u16(data, at + 2 * j) for j in range(6)]
    return pressure


def _header(data: bytes | bytearray, header: int) -> SensorValuesPacket:
    serial_number = u16(data, 1)
    timestamp = to_timestamp(data[3], data[4], data[5], data[6], data[7])
    return SensorValuesPacket(header, serial_number, timestamp)


def _parse_mode1(data: bytes | bytearray, sensor_range: Range) -> SensorValuesPacket:
    """ヘッダ 50: 21 バイト間隔で quat(8) gyro(6) acc(6) 経過ms(1) が 4 サンプル"""
    packet = _header(data, HEADER_QUAT_GYRO_ACC)
    each_timestamp = packet.timestamp
    for i in range(3, -1, -1):
        step = 21 * i
        acc, converted_acc, gyro, converted_gyro = _imu(data, 16 + step, 22 + step, sensor_range)
        quat = _quat(data, 8 + step)
        if i != 3:
            each_timestamp = each_timestamp + data[28 + step]
        stamp([acc, converted_acc, gyro, converted_gyro, quat],
              each_timestamp, packet.serial_number, 3 - i)
        packet.frames.append(SensorFrame(acc, converted_acc, gyro, converted_gyro, quat=quat))
    return packet


def _parse_mode3(data: bytes | bytearray, sensor_range: Range) -> SensorValuesPacket:
    """
    ヘッダ 55: 24 バイト間隔で gyro(6) acc(6) 圧力(12) が 4 サンプル

    経過 ms は公開版と同じく data[28 + step] から足している（ヘッダ 50 と同じ位置。
    このレイアウトでは圧力のバイトにあたる）。
    """
    packet = _header(data, HEADER_GYRO_ACC_PRESS)
    each_timestamp = packet.timestamp
    for i in range(3, -1, -1):
        step = 24 * i
        acc, converted_acc, gyro, converted_gyro = _imu(data, 8 + step, 14 + step, sensor_range)
        pressure = _pressure(data, 20 + step)
        if i != 3:
            each_timestamp = each_timestamp + data[28 + step]
        stamp([acc, converted_acc, gyro, converted_gyro, pressure],
              each_timestamp, packet.serial_number, 3 - i)
        packet.frames.append(SensorFrame(acc, converted_acc, gyro, converted_gyro, pressure=pressure))
    return packet


def _parse_mode4(data: bytes | bytearray, sensor_range: Range) -> SensorValuesPacket:
    """ヘッダ 56: 32 バイト間隔で quat(8) gyro(6) acc(6) 圧力(12) が 2 サンプル（同じタイムスタンプ）"""
    packet = _header(data, HEADER_QUAT_GYRO_ACC_PRESS)
    for i in range(1, -1, -1):
        step = 32 * i
        acc, converted_acc, gyro, converted_gyro = _imu(data, 16 + step, 22 + step, sensor_range)
        quat = _quat(data, 8 + step)
        pressure = _pressure(data, 28 + step)
        stamp([acc, converted_acc, gyro, converted_gyro, quat, pressure],
              packet.timestamp, packet.serial_number, 1 - i)
        packet.frames.append(SensorFrame(acc, converted_acc, gyro, converted_gyro,
                                         quat=quat, pressure=pressure))
    return packet


class InsoleProfile:
    """ORPHE INSOLE"""

    kind: DeviceKind = 'insole'
    label = 'ORPHE INSOLE'
    write_wait_sec = WRITE_WAIT_INTERVAL_SEC
    lost_data_headers = frozenset({HEADER_QUAT_GYRO_ACC, HEADER_GYRO_ACC_PRESS, HEADER_QUAT_GYRO_ACC_PRESS})

    def matches_name(self, name: str | None) -> bool:
        return name is not None and "INS" in name

    def parse_device_information(self, data: bytes | bytearray) -> InsoleDeviceInformation:
        return InsoleDeviceInformation(data)

    def describe_device_information(self, di: InsoleDeviceInformation) -> list[str]:  # type: ignore[override]
        return [
            f"Version: {di.version}",
            f"Battery: {di.battery}",
            f"Mount Position: {di.mount_position}",
            f"ACC Range: {di.range.acc}",
            f"GYRO Range: {di.range.gyro}",
        ]

    def build_range_write(self, di: InsoleDeviceInformation) -> bytearray:  # type: ignore[override]
        return build_device_information_write(di)

    def parse_sensor_values(self, data: bytes | bytearray, sensor_range: Range) -> SensorValuesPacket | None:
        if data[0] == HEADER_QUAT_GYRO_ACC:
            return _parse_mode1(data, sensor_range)
        if data[0] == HEADER_GYRO_ACC_PRESS:
            return _parse_mode3(data, sensor_range)
        if data[0] == HEADER_QUAT_GYRO_ACC_PRESS:
            return _parse_mode4(data, sensor_range)
        return None


def insole_profile() -> InsoleProfile:
    return InsoleProfile()
