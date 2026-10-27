"""ファームウェア情報（GET_FW_NAME キャラクタリスティック）"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import date

FIRMWARE_NAME_BYTE_LENGTH = 23
BUILD_ID_LENGTH = 15

# FIFO（ところてん）に対応した FW の最初のリリース日
CORE_FIFO_MIN_RELEASE_DATE = 20260905
INSOLE_FIFO_MIN_RELEASE_DATE = 20260510


@dataclass(frozen=True)
class FirmwareInfo:
    """
    ファームウェア情報

    Attributes:
        build_id: ビルド識別子（機種・用途を表す文字列）
        major, minor, patch: バージョン
        release_date: リリース日の YYYYMMDD（大小比較で前後が分かる）
        name: 表示用のビルド名
    """

    build_id: str
    major: int
    minor: int
    patch: int
    release_date: int

    @property
    def released_at(self) -> date:
        return date(self.release_date // 10000, self.release_date // 100 % 100, self.release_date % 100)

    @property
    def name(self) -> str:
        return f"{self.build_id}-{self.major}-{self.minor:04d}-{self.release_date}-{self.patch:02d}"


def decode_firmware_info(data: bytes | bytearray) -> FirmwareInfo | None:
    """
    GET_FW_NAME の値をデコードする

    前半 15 バイトがビルド識別子の ASCII、後半がバージョンとリリース日。
    長さが足りない・日付が不正（未書き込みや別の形式）なら None。
    """
    if len(data) < FIRMWARE_NAME_BYTE_LENGTH:
        return None
    year = int.from_bytes(data[18:20], 'big')
    month, day = data[20], data[21]
    if not (2000 <= year <= 2199 and 1 <= month <= 12 and 1 <= day <= 31):
        return None
    build_id = ''
    for code in data[:BUILD_ID_LENGTH]:
        if code < 0x20 or code > 0x7E:
            break
        build_id += chr(code)
    return FirmwareInfo(build_id, data[15], int.from_bytes(data[16:18], 'big'), data[22],
                        year * 10000 + month * 100 + day)


def supports_fifo(kind: str, firmware: FirmwareInfo | None) -> bool | None:
    """FIFO に対応した FW か。FW 情報が取れていなければ None（分からない）"""
    if firmware is None:
        return None
    minimum = CORE_FIFO_MIN_RELEASE_DATE if kind == 'core' else INSOLE_FIFO_MIN_RELEASE_DATE
    return firmware.release_date >= minimum


# INSOLE の個体別圧力校正係数を読み出せる FW の最初のリリース日
INSOLE_PRESSURE_CALIBRATION_MIN_RELEASE_DATE = 20260428


def supports_pressure_calibration(firmware: FirmwareInfo | None) -> bool:
    """INSOLE の校正係数を読み出せる FW か（FW 情報が無ければ False）"""
    return firmware is not None and firmware.release_date >= INSOLE_PRESSURE_CALIBRATION_MIN_RELEASE_DATE
