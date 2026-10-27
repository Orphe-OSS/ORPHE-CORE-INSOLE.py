"""デバイスごとの差分（パケットのレイアウト・換算・コマンドのバイト列）"""

from .auto import AutoProfile, auto_profile, detect_device_kind
from .base import ACC_RANGES, GYRO_RANGES, DeviceInformation, DeviceKind, DeviceProfile
from .core import CoreDeviceInformation, CoreProfile, core_profile
from .insole import InsoleDeviceInformation, InsoleProfile, insole_profile

__all__ = [
    'ACC_RANGES',
    'GYRO_RANGES',
    'AutoProfile',
    'CoreDeviceInformation',
    'CoreProfile',
    'DeviceInformation',
    'DeviceKind',
    'DeviceProfile',
    'InsoleDeviceInformation',
    'InsoleProfile',
    'auto_profile',
    'core_profile',
    'detect_device_kind',
    'insole_profile',
]
