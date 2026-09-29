"""
接続したデバイスの名前で CORE / INSOLE を判別する

    device = OrpheCoreInsole()  # profile を省略すると auto
    await device.connect()      # CR-* か INS* の最初に見つかったデバイスにつなぐ
    device.kind                 # 'core' | 'insole'
"""

from __future__ import annotations

from typing import Literal

from .base import DeviceKind, DeviceProfile
from .core import CoreProfile
from .insole import InsoleProfile


def detect_device_kind(name: str | None) -> DeviceKind:
    """デバイス名に `INS` を含めば INSOLE、それ以外（名前が取れない場合を含む）は CORE"""
    return 'insole' if name is not None and "INS" in name else 'core'


class AutoProfile:
    """接続前のプロファイル。接続したデバイスの名前で CoreProfile / InsoleProfile に決まる"""

    kind: Literal['auto'] = 'auto'
    label = 'ORPHE CORE / INSOLE'

    def __init__(self, core: CoreProfile | None = None, insole: InsoleProfile | None = None) -> None:
        self.core = core or CoreProfile()
        self.insole = insole or InsoleProfile()

    def matches_name(self, name: str | None) -> bool:
        return self.core.matches_name(name) or self.insole.matches_name(name)

    def resolve(self, name: str | None) -> DeviceProfile:
        return self.insole if detect_device_kind(name) == 'insole' else self.core


def auto_profile(core: CoreProfile | None = None, insole: InsoleProfile | None = None) -> AutoProfile:
    return AutoProfile(core, insole)
