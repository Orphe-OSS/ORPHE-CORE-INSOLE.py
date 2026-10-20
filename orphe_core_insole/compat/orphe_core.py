"""
ORPHE-CORE.py（orphe_core.py）互換モジュール

    from orphe_core_insole.compat.orphe_core import Orphe

公開版の orphe_core.py と同じ名前・メソッド・コールバックで使える。中身は OrpheCoreInsole。
"""

from __future__ import annotations

from typing import Any

from ..data import (
    AccData,
    GaitData,
    GyroData,
    PronationData,
    QuatData,
    QuatDistanceData,
    Range,
    StepCount,
    StrideData,
    to_timestamp,
)
from ..profiles.core import WRITE_WAIT_INTERVAL_SEC, CoreProfile
from ..profiles.core import CoreDeviceInformation as DeviceInformation
from ..uuids import (
    CHARACTERISTIC_DEVICE_INFORMATION_UUID,
    CHARACTERISTIC_SENSOR_VALUES_UUID,
    CHARACTERISTIC_STEP_ANALYSIS_UUID,
    SERVICE_ORPHE_INFORMATION_UUID,
    SERVICE_OTHER_UUID,
)
from ._legacy import LegacyOrphe, StepAnalysisData, make_sensor_values_data

SensorValuesData = make_sensor_values_data(CoreProfile())

__all__ = [
    'CHARACTERISTIC_DEVICE_INFORMATION_UUID',
    'CHARACTERISTIC_SENSOR_VALUES_UUID',
    'CHARACTERISTIC_STEP_ANALYSIS_UUID',
    'SERVICE_ORPHE_INFORMATION_UUID',
    'SERVICE_OTHER_UUID',
    'WRITE_WAIT_INTERVAL_SEC',
    'AccData',
    'DeviceInformation',
    'GaitData',
    'GyroData',
    'Orphe',
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
    """ORPHE CORE の BLE 通信を行うクラス（公開版互換）"""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(CoreProfile(), **kwargs)

    async def scan_all_devices(self) -> list[Any]:
        """すべての BLE デバイスをスキャンして BLEDevice のリストを返す"""
        print("Scanning all BLE devices...")
        return [device for device, _adv in (await self._discover(5.0)).values()]

    async def set_led(self, is_on: int, pattern: int) -> None:
        try:
            await super().set_led(is_on, pattern)
        except ValueError as error:
            print(error)

    async def set_led_brightness(self, brightness: int) -> None:
        try:
            await super().set_led_brightness(brightness)
        except ValueError as error:
            print(error)

    async def set_lr(self, lr: int) -> None:
        try:
            await super().set_lr(lr)
        except ValueError as error:
            print(error)
