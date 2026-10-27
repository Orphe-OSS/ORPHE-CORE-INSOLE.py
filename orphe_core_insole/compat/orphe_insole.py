"""
ORPHE-INSOLE.py（orphe_insole.py）互換モジュール

    from orphe_core_insole.compat.orphe_insole import Orphe

公開版の orphe_insole.py と同じ名前・メソッド・コールバックで使える。中身は OrpheCoreInsole。
"""

from __future__ import annotations

from typing import Any

from ..data import (
    AccData,
    GaitData,
    GyroData,
    PressureData,
    PronationData,
    QuatData,
    QuatDistanceData,
    Range,
    StepCount,
    StrideData,
    to_timestamp,
)
from ..profiles.base import ACC_RANGES, GYRO_DPS_PER_LSB_PER_RANGE, GYRO_RANGES
from ..profiles.insole import (
    GYRO_SENSITIVITY_SCALE,
    QUAT_SCALE,
    WRITE_WAIT_INTERVAL_SEC,
    InsoleProfile,
    build_device_information_write,
    checksum,
)
from ..profiles.insole import InsoleDeviceInformation as DeviceInformation
from ..uuids import (
    CHARACTERISTIC_DEVICE_INFORMATION_UUID,
    CHARACTERISTIC_SENSOR_VALUES_UUID,
    CHARACTERISTIC_STEP_ANALYSIS_UUID,
    SERVICE_ORPHE_INFORMATION_UUID,
    SERVICE_OTHER_UUID,
)
from ._legacy import LegacyOrphe, StepAnalysisData, make_sensor_values_data

SensorValuesData = make_sensor_values_data(InsoleProfile())

__all__ = [
    'ACC_RANGES',
    'CHARACTERISTIC_DEVICE_INFORMATION_UUID',
    'CHARACTERISTIC_SENSOR_VALUES_UUID',
    'CHARACTERISTIC_STEP_ANALYSIS_UUID',
    'GYRO_DPS_PER_LSB_PER_RANGE',
    'GYRO_RANGES',
    'GYRO_SENSITIVITY_SCALE',
    'QUAT_SCALE',
    'SERVICE_ORPHE_INFORMATION_UUID',
    'SERVICE_OTHER_UUID',
    'WRITE_WAIT_INTERVAL_SEC',
    'AccData',
    'DeviceInformation',
    'GaitData',
    'GyroData',
    'Orphe',
    'PressureData',
    'PronationData',
    'QuatData',
    'QuatDistanceData',
    'Range',
    'SensorValuesData',
    'StepAnalysisData',
    'StepCount',
    'StrideData',
    'to_timestamp',
]


class Orphe(LegacyOrphe):
    """ORPHE INSOLE の BLE 通信を行うクラス（公開版互換）"""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(InsoleProfile(), **kwargs)

    def set_got_pressure_callback(self, callback: Any) -> None:
        """圧力センサの値を取得したときに呼び出されるコールバック関数を設定する"""
        self.got_pressure_callback = callback

    async def scan_all_devices(self) -> dict[str, tuple[Any, Any]]:
        """すべての BLE デバイスをスキャンして {address: (BLEDevice, AdvertisementData)} を返す"""
        print("Scanning all BLE devices...")
        return await self._discover(5.0)

    async def getChecksum(self, data: bytes | bytearray) -> int:
        """device information のチェックサムを計算する"""
        return checksum(data)

    async def right_dec2hex(self, value: int) -> str:
        """10進数を16進数（大文字・4桁ゼロ埋め）にして右2桁を返す"""
        return format(value, '04X')[-2:]

    async def write_device_information(self, data: bytes | bytearray | None = None) -> None:
        """self.device_information の内容を書き込む。書き込むバイト列とチェックサムを表示する"""
        if data is None:
            if self.device_information is None:
                print("Device information is not set, Please read device information first.")
                return
            data = build_device_information_write(self.device_information)
            print(f"data: {list(data)}, size: {len(data)}")
            print(f"checksum: {data[19]}, hex: {await self.right_dec2hex(data[19])}")
        await super().write_device_information(data)

    async def set_mount_position(self, mount_position: int) -> None:
        try:
            await super().set_mount_position(mount_position)
        except ValueError as error:
            print(error)

    async def set_data_streaming_mode(self, mode: int = 4) -> None:
        try:
            await super().set_data_streaming_mode(mode)
        except ValueError as error:
            print(error)
