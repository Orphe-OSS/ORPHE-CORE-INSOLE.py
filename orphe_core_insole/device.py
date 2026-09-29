"""
OrpheCoreInsole — ORPHE CORE / ORPHE INSOLE 共通の BLE 通信（bleak）

スキャン・接続・切断監視・device information の読み書き・notify の購読と、
コールバックへの配送を受け持つ。デバイスごとの差分は profiles/ に閉じている。

    device = OrpheCoreInsole()                 # CORE / INSOLE を名前で自動判別
    device.on('converted_acc', lambda acc: acc.print())
    if await device.connect():
        await device.start_sensor_values_notification()
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Protocol

from .data import PressureData, StepCount
from .firmware import FirmwareInfo, decode_firmware_info, supports_pressure_calibration
from .packets import parse_step_analysis
from .pressure import (
    PRESSURE_SENSOR_COUNT,
    PressureCalibration,
    decode_pressure_calibration,
    encode_pressure_calibration_request,
    is_pressure_calibration_packet,
    pressure_to_newton,
)
from .profiles import core as core_protocol
from .profiles import insole as insole_protocol
from .profiles.auto import AutoProfile
from .profiles.base import DeviceInformation, DeviceProfile, acc_range_index, gyro_range_index
from .time_sync import (
    DEFAULT_TIME_SYNC_SAMPLES,
    DeviceDateTime,
    SyncTimeResult,
    encode_date_time,
    read_date_time,
    sync_device_time,
)
from .uuids import (
    CHARACTERISTIC_DATE_TIME_UUID,
    CHARACTERISTIC_DEVICE_INFORMATION_UUID,
    CHARACTERISTIC_FIRMWARE_NAME_UUID,
    CHARACTERISTIC_SENSOR_VALUES_UUID,
    CHARACTERISTIC_STEP_ANALYSIS_UUID,
)

# on() で購読できるイベント
SENSOR_EVENTS = ('acc', 'converted_acc', 'gyro', 'converted_gyro', 'pressure', 'converted_pressure', 'quat')
STEP_ANALYSIS_EVENTS = ('gait', 'stride', 'pronation', 'quat_distance')
EVENTS = SENSOR_EVENTS + ('sensor_values',) + STEP_ANALYSIS_EVENTS + ('lost_data', 'disconnect')

# スキャン時間[s]（bleak の既定値と同じ）
DEFAULT_SCAN_TIMEOUT_SEC = 5.0
# 切断を監視する間隔[s]
MONITOR_INTERVAL_SEC = 1.0


class UnsupportedOperationError(RuntimeError):
    """接続中のデバイスでは使えない操作（INSOLE に LED を設定するなど）"""


class BleClient(Protocol):
    """OrpheCoreInsole が使う BleakClient の機能"""

    @property
    def is_connected(self) -> bool: ...

    async def connect(self) -> Any: ...

    async def disconnect(self) -> Any: ...

    async def read_gatt_char(self, char_specifier: str) -> bytearray: ...

    async def write_gatt_char(self, char_specifier: str, data: bytes | bytearray) -> None: ...

    async def start_notify(self, char_specifier: str, callback: Callable[..., Any]) -> None: ...

    async def stop_notify(self, char_specifier: str) -> None: ...


# address -> (BLEDevice, AdvertisementData)
Discover = Callable[[float], Awaitable[dict[str, tuple[Any, Any]]]]
ClientFactory = Callable[[Any], BleClient]


async def _bleak_discover(timeout: float) -> dict[str, tuple[Any, Any]]:
    from bleak import BleakScanner
    return await BleakScanner.discover(timeout=timeout, return_adv=True)


def _bleak_client(device: Any) -> BleClient:
    from bleak import BleakClient
    return BleakClient(device)


@dataclass
class ScanResult:
    """
    スキャンで見つかったデバイス

    Attributes:
        name: デバイス名
        address: アドレス（macOS では UUID）。connect() に渡せる
        rssi: 受信強度
        kind: 'core' / 'insole'。ORPHE のデバイスでなければ None
        device: bleak の BLEDevice
    """

    name: str | None
    address: str
    rssi: int | None
    kind: str | None
    device: Any


class OrpheCoreInsole:
    """
    ORPHE CORE / ORPHE INSOLE の BLE 通信を行うクラス

    Args:
        profile: core_profile() / insole_profile()。省略すると auto_profile() で、
            接続したデバイスの名前で判別する
        verbose: スキャンや接続の経過を標準出力に表示する
        discover: スキャン関数（テスト用。既定は bleak の BleakScanner.discover）
        client_factory: BLEDevice から BleakClient を作る関数（テスト用）
        time_sync_samples: connect() でデバイスの時計を合わせるときに往復時間を測る回数。0 なら合わせない
    """

    def __init__(
        self,
        profile: DeviceProfile | AutoProfile | None = None,
        *,
        verbose: bool = True,
        discover: Discover | None = None,
        client_factory: ClientFactory | None = None,
        monitor_interval_sec: float = MONITOR_INTERVAL_SEC,
        time_sync_samples: int = DEFAULT_TIME_SYNC_SAMPLES,
    ) -> None:
        self._requested_profile: DeviceProfile | AutoProfile = profile or AutoProfile()
        self.profile: DeviceProfile | AutoProfile = self._requested_profile
        self.verbose = verbose
        self.client: BleClient | None = None
        # 接続したデバイスの名前とアドレス
        self.name: str | None = None
        self.address: str | None = None
        self.device_information: Any = None
        self.serial_number_prev = 0
        self.step_count = StepCount()
        self._listeners: dict[str, list[Callable[..., Any]]] = {event: [] for event in EVENTS}
        self._discover = discover or _bleak_discover
        self._client_factory = client_factory or _bleak_client
        self._monitor_interval_sec = monitor_interval_sec
        self.time_sync_samples = time_sync_samples
        # connect() で時計を合わせた結果
        self.time_sync: SyncTimeResult | None = None
        self._monitor_task: asyncio.Task[None] | None = None
        # INSOLE の streaming mode（このクラスで最後に設定した値。FIFO の復帰先に使う）
        self.streaming_mode: int | None = None
        # read_firmware_info() で読んだファームウェア情報
        self.firmware: FirmwareInfo | None = None
        # SENSOR_VALUES の notify を購読しているか
        self.sensor_values_notifying = False
        # notify を横取りする関数（キャラクタリスティックの UUID → 関数。FIFO 収録・歩行解析で使う）
        self._notify_sinks: dict[str, Callable[[bytes], None]] = {}
        # INSOLE の個体別圧力校正係数（index = ch 0..5。係数が無い ch は None）。読んでいなければ None
        self.pressure_calibrations: list[PressureCalibration | None] | None = None
        # 校正係数の応答待ち（ch → Future）
        self._pending_calibrations: dict[int, asyncio.Future[PressureCalibration]] = {}
        # 校正係数を読む間だけ、センサー値のイベントを止める
        self._mute_sensor_events = False

    # ── イベント ──────────────────────────────────────────────

    @property
    def kind(self) -> str:
        """'core' / 'insole'。自動判別で接続前なら 'auto'"""
        return self.profile.kind

    def on(self, event: str, callback: Callable[..., Any]) -> Callable[[], None]:
        """
        イベントを購読する。戻り値を呼ぶと購読をやめる

        センサー値: acc, converted_acc, gyro, converted_gyro, quat, pressure（INSOLE、ADC 値）,
            converted_pressure（INSOLE、[N]。read_pressure_calibrations() で読んだ係数を使う）
        パケット単位: sensor_values（SensorValuesPacket。そのパケットのサンプルを通知したあと）
        歩行解析: gait, stride, pronation, quat_distance
        その他: lost_data（引数は直前と今回のシリアル番号）, disconnect（引数なし）
        """
        if event not in self._listeners:
            raise ValueError(f"unknown event: {event!r} (choose from {', '.join(EVENTS)})")
        self._listeners[event].append(callback)
        return lambda: self.off(event, callback)

    def off(self, event: str, callback: Callable[..., Any]) -> None:
        listeners = self._listeners.get(event, [])
        if callback in listeners:
            listeners.remove(callback)

    def _emit(self, event: str, *args: Any) -> None:
        for callback in list(self._listeners[event]):
            callback(*args)

    def _log(self, message: str) -> None:
        if self.verbose:
            print(message)

    # ── 接続 ──────────────────────────────────────────────────

    async def scan(self, timeout: float = DEFAULT_SCAN_TIMEOUT_SEC,
                   all_devices: bool = False) -> list[ScanResult]:
        """
        周りの BLE デバイスをスキャンする

        Args:
            timeout: スキャン時間[s]
            all_devices: True なら ORPHE 以外のデバイスも返す
        """
        auto = AutoProfile()
        results = []
        for device, adv in (await self._discover(timeout)).values():
            if not all_devices and not self._requested_profile.matches_name(device.name):
                continue
            kind = None
            if auto.core.matches_name(device.name):
                kind = 'core'
            elif auto.insole.matches_name(device.name):
                kind = 'insole'
            results.append(ScanResult(device.name, device.address, adv.rssi, kind, device))
        return results

    async def connect(self, address: str | None = None,
                      timeout: float = DEFAULT_SCAN_TIMEOUT_SEC) -> bool:
        """
        スキャンしてデバイスに接続する

        Args:
            address: 接続するデバイスのアドレス。省略するとプロファイルに合う名前の
                最初に見つかったデバイスに接続する（CORE は CR-*、INSOLE は INS*）
            timeout: スキャン時間[s]
        Returns:
            接続できたら True
        """
        self._log(f"Scanning for {self._requested_profile.label} BLE device...[address specified: {address}]")
        target = None
        for device, _adv in (await self._discover(timeout)).values():
            if address is not None:
                if device.address == address:
                    target = device
                    break
            elif self._requested_profile.matches_name(device.name):
                target = device
                break

        if target is None:
            self._log("Target device not found.")
            return False
        self._log(f"Found target device: {target.name}(name), {target.address}(address)")

        if isinstance(self._requested_profile, AutoProfile):
            self.profile = self._requested_profile.resolve(target.name)

        self.name = target.name
        self.address = target.address
        self.client = self._client_factory(target)
        await self.client.connect()
        if not self.client.is_connected:
            self._log("Failed to connect to the device")
            return False

        self._log(f"Connected to the device ({self.profile.label})")
        if self.time_sync_samples > 0:
            try:
                self.time_sync = await self.sync_device_time(self.time_sync_samples)
            except BaseException:
                await self.client.disconnect()
                raise
        self._monitor_task = asyncio.create_task(self.monitor_connection(self.client))
        return True

    async def monitor_connection(self, client: BleClient) -> None:
        """接続状態を定期的に確認し、切れたら disconnect イベントを出す"""
        while True:
            await asyncio.sleep(self._monitor_interval_sec)
            if not client.is_connected:
                self._emit('disconnect')
                break

    def is_connected(self) -> bool:
        if self.client is None:
            return False
        return self.client.is_connected

    async def disconnect(self) -> None:
        await self._require_client().disconnect()

    def _require_client(self) -> BleClient:
        if self.client is None:
            raise RuntimeError("Not connected. Call connect() first.")
        return self.client

    def _require_kind(self, kind: str, operation: str) -> None:
        if self.profile.kind != kind:
            raise UnsupportedOperationError(
                f"{operation}() is only available for ORPHE {kind.upper()} (connected: {self.profile.kind})")

    def _resolved_profile(self) -> DeviceProfile:
        profile = self.profile
        if isinstance(profile, AutoProfile):
            raise RuntimeError("Device kind is not determined yet. Call connect() first.")  # noqa: TRY004
        return profile

    # ── device information ───────────────────────────────────

    async def read_device_information(self) -> Any:
        """
        device information を読む。読んだ値は self.device_information にも保存する

        Returns:
            CoreDeviceInformation / InsoleDeviceInformation
        """
        data = await self._require_client().read_gatt_char(CHARACTERISTIC_DEVICE_INFORMATION_UUID)
        self.device_information = self._resolved_profile().parse_device_information(data)
        return self.device_information

    async def print_device_information(self) -> None:
        """device information を読んで標準出力に表示する（デバッグ用）"""
        di = await self.read_device_information()
        for line in self._resolved_profile().describe_device_information(di):
            print(line)

    async def write_device_information(self, data: bytes | bytearray | None = None) -> None:
        """
        device information に書き込む

        書き込み直後に読み直すと正しい値が返らないため、プロファイルの write_wait_sec だけ待つ。

        Args:
            data: 書き込むバイト列。省略すると self.device_information の内容を書き込む
        """
        if data is None:
            if self.device_information is None:
                raise RuntimeError("Device information is not set, Please read device information first.")
            data = self._resolved_profile().build_range_write(self.device_information)
        await self._write(data)

    async def _write(self, data: bytes | bytearray) -> None:
        await self.write_command(data)
        await asyncio.sleep(self._resolved_profile().write_wait_sec)

    async def write_command(self, data: bytes | bytearray) -> None:
        """DEVICE_INFORMATION にコマンドを書き込む（待ちなし。FIFO のコマンドに使う）"""
        await self._require_client().write_gatt_char(CHARACTERISTIC_DEVICE_INFORMATION_UUID, data)

    def max_write_size(self) -> int | None:
        """
        DEVICE_INFORMATION に 1 回で書き込める長さ[byte]。bleak が分からなければ None

        DEVICE_INFORMATION は応答なし書き込みなので、MTU - 3 を超える書き込みは届かない（macOS は黙って捨てる）。
        """
        services = getattr(self.client, 'services', None)
        if services is None:
            return None
        try:
            characteristic = services.get_characteristic(CHARACTERISTIC_DEVICE_INFORMATION_UUID)
            return int(characteristic.max_write_without_response_size) if characteristic else None
        except Exception:  # noqa: BLE001 (bleak の版やバックエンドによって取れない)
            return None

    async def read_firmware_info(self) -> FirmwareInfo | None:
        """
        ファームウェア情報を読む。読んだ値は self.firmware にも保存する

        Returns:
            FirmwareInfo。キャラクタリスティックが無い・形式が違う FW では None
        """
        client = self._require_client()
        try:
            data = await client.read_gatt_char(CHARACTERISTIC_FIRMWARE_NAME_UUID)
        except Exception:  # noqa: BLE001 (古い FW にはキャラクタリスティックが無い)
            data = b''
        self.firmware = decode_firmware_info(data)
        return self.firmware

    # ── 時刻 ─────────────────────────────────────────────────

    async def read_date_time(self) -> DeviceDateTime:
        """デバイスの時刻を読む（往復時間も返す）"""
        client = self._require_client()
        return await read_date_time(lambda: client.read_gatt_char(CHARACTERISTIC_DATE_TIME_UUID))

    async def write_date_time(self, date: datetime) -> None:
        """デバイスの時刻を書き込む"""
        await self._require_client().write_gatt_char(CHARACTERISTIC_DATE_TIME_UUID, encode_date_time(date))

    async def sync_device_time(self, samples: int = DEFAULT_TIME_SYNC_SAMPLES) -> SyncTimeResult:
        """
        デバイスの時計を PC の時刻に合わせる（connect() でも行う）

        時刻を samples 回読んで往復時間を測り、「PC 時刻 + 平均往復時間/2」を書き込む。
        """
        client = self._require_client()
        return await sync_device_time(
            lambda: client.read_gatt_char(CHARACTERISTIC_DATE_TIME_UUID),
            lambda data: client.write_gatt_char(CHARACTERISTIC_DATE_TIME_UUID, data),
            samples,
        )

    async def set_acc_range(self, acc_range: int) -> None:
        """加速度レンジを 2, 4, 8, 16[G] のいずれかに設定する"""
        index = acc_range_index(acc_range)
        di: DeviceInformation = await self.read_device_information()
        di.range.acc = index
        await self.write_device_information()

    async def set_gyro_range(self, gyro_range: int) -> None:
        """ジャイロレンジを 250, 500, 1000, 2000[deg/s] のいずれかに設定する"""
        index = gyro_range_index(gyro_range)
        di: DeviceInformation = await self.read_device_information()
        di.range.gyro = index
        await self.write_device_information()

    # ── CORE だけのコマンド ───────────────────────────────────

    async def set_led(self, is_on: int, pattern: int) -> None:
        """
        LED を点灯・消灯する（CORE）

        Args:
            is_on: 0 or 1
            pattern: 0-4
        """
        self._require_kind('core', 'set_led')
        await self._write(core_protocol.build_led_write(is_on, pattern))

    async def set_led_brightness(self, brightness: int) -> None:
        """LED の明るさを 0-255 で設定する（CORE）。ほかの設定は読み直した値のまま書き込む"""
        self._require_kind('core', 'set_led_brightness')
        core_protocol.validate_led_brightness(brightness)
        di = await self.read_device_information()
        await self._write(core_protocol.build_device_information_write(di, led=brightness))

    async def set_lr(self, lr: int) -> None:
        """左右を 0 or 1 で設定する（CORE）"""
        self._require_kind('core', 'set_lr')
        core_protocol.validate_lr(lr)
        di = await self.read_device_information()
        await self._write(core_protocol.build_device_information_write(di, lr=lr))

    # ── INSOLE だけのコマンド ─────────────────────────────────

    async def set_mount_position(self, mount_position: int) -> None:
        """
        マウント位置を設定する（INSOLE）

        Args:
            mount_position: 0=足底・左、1=足底・右、2=足背・左、3=足背・右
        """
        self._require_kind('insole', 'set_mount_position')
        insole_protocol.validate_mount_position(mount_position)
        di = await self.read_device_information()
        di.mount_position = mount_position
        await self.write_device_information()

    async def set_data_streaming_mode(self, mode: int = 4) -> None:
        """
        データストリーミングモードを設定する（INSOLE）

        Args:
            mode: 1（quat・gyro・acc 200Hz）、3（gyro・acc・圧力 200Hz）、
                4（quat・gyro・acc・圧力 100Hz）
        """
        self._require_kind('insole', 'set_data_streaming_mode')
        await self._write(insole_protocol.build_streaming_mode_write(mode))
        self.streaming_mode = mode

    # ── notify ────────────────────────────────────────────────

    async def start_sensor_values_notification(self) -> None:
        """センサー値の通知を開始する。換算にレンジが要るので先に device information を読む"""
        await self.read_device_information()
        await self._require_client().start_notify(
            CHARACTERISTIC_SENSOR_VALUES_UUID, self.sensor_values_notification_handler)
        self.sensor_values_notifying = True

    async def stop_sensor_values_notification(self) -> None:
        await self._require_client().stop_notify(CHARACTERISTIC_SENSOR_VALUES_UUID)
        self.sensor_values_notifying = False

    def set_notify_sink(self, sink: Callable[[bytes], None],
                        uuid: str = CHARACTERISTIC_SENSOR_VALUES_UUID) -> Callable[[], None]:
        """
        notify を横取りする。横取り中はそのキャラクタリスティックのイベント（acc や gait など）を出さない

        Args:
            sink: notify の値を受け取る関数
            uuid: 横取りするキャラクタリスティック（既定は SENSOR_VALUES。STEP_ANALYSIS も指定できる）
        Returns:
            横取りをやめる関数
        Raises:
            RuntimeError: ほかの処理が横取り中
        """
        if uuid in self._notify_sinks:
            raise RuntimeError(f"notify of {uuid} is already taken by another consumer")
        self._notify_sinks[uuid] = sink

        def remove() -> None:
            if self._notify_sinks.get(uuid) is sink:
                del self._notify_sinks[uuid]

        return remove

    async def start_step_analysis_notification(self) -> None:
        """歩行解析（STEP_ANALYSIS）の通知を開始する"""
        await self._require_client().start_notify(
            CHARACTERISTIC_STEP_ANALYSIS_UUID, self.step_analysis_notification_handler)

    async def stop_step_analysis_notification(self) -> None:
        await self._require_client().stop_notify(CHARACTERISTIC_STEP_ANALYSIS_UUID)

    async def sensor_values_notification_handler(self, sender: Any, data: bytearray) -> None:
        if is_pressure_calibration_packet(data):
            self._receive_pressure_calibration(data)
            return
        sink = self._notify_sinks.get(CHARACTERISTIC_SENSOR_VALUES_UUID)
        if sink is not None:
            sink(bytes(data))
            return
        self.handle_sensor_values(data)

    async def step_analysis_notification_handler(self, sender: Any, data: bytearray) -> None:
        sink = self._notify_sinks.get(CHARACTERISTIC_STEP_ANALYSIS_UUID)
        if sink is not None:
            sink(bytes(data))
            return
        self.handle_step_analysis(data)

    def handle_sensor_values(self, data: bytes | bytearray) -> None:
        """
        SENSOR_VALUES パケットを 1 つ処理する

        サンプルごとに acc, converted_acc, gyro, converted_gyro, pressure, quat の順で通知し、
        パケット全体を sensor_values で通知する。最後にシリアル番号が飛んでいれば lost_data を通知する。
        """
        if not self.is_connected() or isinstance(self.profile, AutoProfile) or self._mute_sensor_events:
            return
        packet = self.profile.parse_sensor_values(data, self.device_information.range)
        if packet is None:
            return

        for frame in packet.frames:
            self._emit('acc', frame.acc)
            self._emit('converted_acc', frame.converted_acc)
            self._emit('gyro', frame.gyro)
            self._emit('converted_gyro', frame.converted_gyro)
            if frame.pressure is not None:
                self._emit('pressure', frame.pressure)
                if self._listeners['converted_pressure']:
                    self._emit('converted_pressure', self.convert_pressure(frame.pressure))
            if frame.quat is not None:
                self._emit('quat', frame.quat)
        self._emit('sensor_values', packet)

        if packet.header in self.profile.lost_data_headers and packet.serial_number is not None:
            if packet.serial_number - self.serial_number_prev > 1:
                self._emit('lost_data', self.serial_number_prev, packet.serial_number)
            self.serial_number_prev = packet.serial_number

    # ── 圧力の校正係数（INSOLE） ─────────────────────────────

    async def read_pressure_calibrations(self, timeout_sec: float = 1.0,
                                         retries: int = 3) -> list[PressureCalibration | None] | None:
        """
        INSOLE の個体別圧力校正係数を 6ch ぶん読み、self.pressure_calibrations に保存する（INSOLE）

        FW が圧力を含む streaming mode（3 / 4）のときだけ応答するので、それ以外なら読む間だけ 4 にする。
        応答が無い ch と、FW に値が書かれていない ch は None（その ch は固定式で換算する）。

        Returns:
            ch ごとの係数。FW が非対応（リリース日 20260428 より前）・FW 情報が読めないときは None
        """
        self._require_kind('insole', 'read_pressure_calibrations')
        self.pressure_calibrations = None
        firmware = self.firmware or await self.read_firmware_info()
        if not supports_pressure_calibration(firmware):
            return None

        previous_mode = self.streaming_mode
        if previous_mode not in (3, 4):
            await self.set_data_streaming_mode(4)
        started_notify = not self.sensor_values_notifying
        if started_notify:
            self._mute_sensor_events = True
            await self.start_sensor_values_notification()
        try:
            calibrations: list[PressureCalibration | None] = []
            for sensor_index in range(PRESSURE_SENSOR_COUNT):
                received = None
                for _ in range(retries):
                    received = await self._request_pressure_calibration(sensor_index, timeout_sec)
                    if received is not None:
                        break
                calibrations.append(None if received is None or received.is_placeholder else received)
        finally:
            if started_notify:
                await self.stop_sensor_values_notification()
                self._mute_sensor_events = False
            if previous_mode is not None and previous_mode not in (3, 4):
                await self.set_data_streaming_mode(previous_mode)
        self.pressure_calibrations = calibrations
        return calibrations

    async def _request_pressure_calibration(self, sensor_index: int, timeout_sec: float) -> PressureCalibration | None:
        future: asyncio.Future[PressureCalibration] = asyncio.get_running_loop().create_future()
        self._pending_calibrations[sensor_index] = future
        try:
            await self.write_command(encode_pressure_calibration_request(sensor_index))
            return await asyncio.wait_for(future, timeout_sec)
        except asyncio.TimeoutError:
            return None
        finally:
            if self._pending_calibrations.get(sensor_index) is future:
                del self._pending_calibrations[sensor_index]

    def _receive_pressure_calibration(self, data: bytes | bytearray) -> None:
        decoded = decode_pressure_calibration(data)
        if decoded is None:
            return
        sensor_index, calibration = decoded
        future = self._pending_calibrations.get(sensor_index)
        if future is not None and not future.done():
            future.set_result(calibration)

    def convert_pressure(self, pressure: PressureData) -> PressureData:
        """
        圧力の ADC 値を [N] にした PressureData を返す

        read_pressure_calibrations() で読んだ係数がある ch はそれを使い、無い ch は固定式
        （mV に直して c1·exp(c2·mV) + c3）で換算する。
        """
        calibrations = self.pressure_calibrations
        converted = PressureData()
        converted.values = [pressure_to_newton(raw, calibrations[i] if calibrations else None)
                            for i, raw in enumerate(pressure.values)]
        converted.timestamp = pressure.timestamp
        converted.serial_number = pressure.serial_number
        converted.packet_number = pressure.packet_number
        return converted

    def handle_step_analysis(self, data: bytes | bytearray) -> None:
        """STEP_ANALYSIS パケットを 1 つ処理する"""
        result = parse_step_analysis(data, self.step_count)
        if result is not None:
            self._emit(*result)
