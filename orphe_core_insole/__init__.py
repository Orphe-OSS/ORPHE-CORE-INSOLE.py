"""
orphe_core_insole — ORPHE CORE / ORPHE INSOLE を Python（bleak）で扱う SDK

    from orphe_core_insole import OrpheCoreInsole

    device = OrpheCoreInsole()   # CORE / INSOLE をデバイス名で自動判別
    device.on('converted_acc', lambda acc: acc.print())
    if await device.connect():
        await device.start_sensor_values_notification()

ORPHE-CORE.py / ORPHE-INSOLE.py の既存コードは orphe_core_insole.compat から import する。
"""

from .data import (
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
)
from .device import EVENTS, OrpheCoreInsole, ScanResult, UnsupportedOperationError
from .fifo import FifoRecorder, FifoSample, FifoTiming
from .firmware import FirmwareInfo
from .gait import GaitMotionPacket, GaitRow, InsoleGait
from .packets import SensorFrame, SensorValuesPacket, parse_step_analysis
from .profiles import (
    ACC_RANGES,
    GYRO_RANGES,
    AutoProfile,
    CoreDeviceInformation,
    CoreProfile,
    DeviceKind,
    DeviceProfile,
    InsoleDeviceInformation,
    InsoleProfile,
    auto_profile,
    core_profile,
    detect_device_kind,
    insole_profile,
)
from .time_sync import DeviceDateTime, SyncTimeResult
from .uuids import (
    CHARACTERISTIC_DATE_TIME_UUID,
    CHARACTERISTIC_DEVICE_INFORMATION_UUID,
    CHARACTERISTIC_SENSOR_VALUES_UUID,
    CHARACTERISTIC_STEP_ANALYSIS_UUID,
    SERVICE_ORPHE_INFORMATION_UUID,
    SERVICE_OTHER_UUID,
)

__version__ = '0.1.0'

__all__ = [
    'ACC_RANGES',
    'CHARACTERISTIC_DATE_TIME_UUID',
    'CHARACTERISTIC_DEVICE_INFORMATION_UUID',
    'CHARACTERISTIC_SENSOR_VALUES_UUID',
    'CHARACTERISTIC_STEP_ANALYSIS_UUID',
    'EVENTS',
    'GYRO_RANGES',
    'SERVICE_ORPHE_INFORMATION_UUID',
    'SERVICE_OTHER_UUID',
    'AccData',
    'AutoProfile',
    'CoreDeviceInformation',
    'CoreProfile',
    'DeviceDateTime',
    'DeviceKind',
    'DeviceProfile',
    'FifoRecorder',
    'FifoSample',
    'FifoTiming',
    'FirmwareInfo',
    'GaitData',
    'GaitMotionPacket',
    'GaitRow',
    'GyroData',
    'InsoleDeviceInformation',
    'InsoleGait',
    'InsoleProfile',
    'OrpheCoreInsole',
    'PressureData',
    'PronationData',
    'QuatData',
    'QuatDistanceData',
    'Range',
    'ScanResult',
    'SensorFrame',
    'SensorValuesPacket',
    'StepCount',
    'StrideData',
    'SyncTimeResult',
    'UnsupportedOperationError',
    'auto_profile',
    'core_profile',
    'detect_device_kind',
    'insole_profile',
    'parse_step_analysis',
]
