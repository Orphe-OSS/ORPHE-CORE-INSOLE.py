# ORPHE Core-Insole.py

ORPHE CORE / ORPHE INSOLE を Python（[bleak](https://github.com/hbldh/bleak)）で扱う SDK。

```python
import asyncio
from orphe_core_insole import OrpheCoreInsole

async def main():
    device = OrpheCoreInsole()                        # CORE / INSOLE をデバイス名で自動判別
    device.on('converted_acc', lambda acc: acc.print())
    device.on('pressure', lambda pressure: pressure.print())  # INSOLE のみ
    if await device.connect():
        await device.start_sensor_values_notification()
        await asyncio.sleep(10)
        await device.disconnect()

asyncio.run(main())
```

## 特長

- CORE / INSOLE を同じクラス・同じコールバックで扱える。デバイス差分は `orphe_core_insole/profiles/` だけにある
- 接続したデバイスの名前で CORE / INSOLE を自動判別（`CR-*` は CORE、`INS*` は INSOLE）

## 動作環境

- Python 3.10 以上
- bleak 0.22 以上（macOS / Windows / Linux）

## インストール

```bash
git clone <このリポジトリ>
cd orphe-device-py
pip install -e .                  # サンプルも動かすなら pip install -e ".[examples]"
```

[uv](https://docs.astral.sh/uv/) を使う場合は `uv sync` でも入ります（uv は pip では入らないので、[別途インストール](https://docs.astral.sh/uv/getting-started/installation/)してください）。

## 開発

[uv](https://docs.astral.sh/uv/) を使う場合（uv は別途インストールしてください。[インストール方法](https://docs.astral.sh/uv/getting-started/installation/)）:

```bash
uv sync
uv run python -m unittest discover -s tests
uv run orphe-core-insole scan
```

pip だけの場合:

```bash
pip install -e .
python -m unittest discover -s tests
orphe-core-insole scan
```

テストは BLE なしで動きます（bleak の代わりに偽物のクライアントを使う）。

## コマンド

インストールすると `orphe-core-insole` コマンドが使えます（`python -m orphe_core_insole` でも同じ）。

```bash
orphe-core-insole scan                          # 周りの CORE / INSOLE を探す（--all で全デバイス）
orphe-core-insole info                          # device information を表示（INSOLE は圧力校正係数も）
orphe-core-insole set --acc 16 --gyro 2000      # 設定（CORE: --lr / --led-brightness / --led、INSOLE: --mount / --mode）
orphe-core-insole stream --csv out.csv          # センサー値を受信。受信数/秒と欠損を毎秒表示（--print でサンプルも表示。INSOLE の圧力は ADC 値と N）
orphe-core-insole stream --mode 3 --duration 10 # INSOLE の streaming mode を指定して 10 秒
orphe-core-insole fifo --csv out.csv            # FIFO で欠損なく収録（Ctrl+C か --duration で終了）
orphe-core-insole step                          # 歩行解析（INSOLE は --csv で 1 歩ずつ CSV に保存）
orphe-core-insole check                         # 実機確認（--write で設定の書き込みも確かめる）
```

どのサブコマンドも `--device core|insole|auto`（既定 auto）、`--address`、`--timeout`（スキャン秒数）を指定できます。
接続の経過と要約は標準エラー、`--print` のデータは標準出力に出ます。

## 使い方

### 接続と購読

```python
from orphe_core_insole import OrpheCoreInsole, core_profile, insole_profile

device = OrpheCoreInsole()                  # 自動判別
# device = OrpheCoreInsole(core_profile())  # CORE だけを探す
# device = OrpheCoreInsole(insole_profile())# INSOLE だけを探す

unsubscribe = device.on('quat', lambda quat: quat.print())
device.on('lost_data', lambda prev, cur: print(f"欠損 {prev} -> {cur}"))
device.on('disconnect', lambda: print("切断"))

await device.connect()              # 最初に見つかったデバイス
# await device.connect("XX:XX:...") # アドレス指定（scan() や examples/scan.py で確認）
device.kind                         # 'core' | 'insole'（接続前は 'auto'）

await device.start_sensor_values_notification()
...
unsubscribe()                       # 購読をやめる
await device.stop_sensor_values_notification()
await device.disconnect()
```

`on()` の戻り値を呼ぶか `off(event, callback)` で購読をやめます。

### イベント

| イベント | 引数 | デバイス |
|---|---|---|
| `acc` / `gyro` | `AccData` / `GyroData`（raw / 32768） | 両方 |
| `converted_acc` / `converted_gyro` | `AccData` [G] / `GyroData` [deg/s] | 両方 |
| `quat` | `QuatData` | 両方（INSOLE の streaming mode 3 は無し） |
| `pressure` | `PressureData`（6ch の ADC 値） | INSOLE |
| `converted_pressure` | `PressureData`（6ch の圧力 [N]。下の「圧力の換算」参照） | INSOLE |
| `gait` / `stride` / `pronation` / `quat_distance` | 歩行解析（`start_step_analysis_notification()`。INSOLE は下の `InsoleGait` がおすすめ） | 両方 |
| `sensor_values` | `SensorValuesPacket`（1 パケットぶんのサンプル。個別のイベントのあとに届く） | 両方 |
| `lost_data` | 直前と今回のシリアル番号 | 両方 |
| `disconnect` | なし | 両方 |

1 パケットに入っている複数サンプルは、サンプルごとに `acc` → `converted_acc` → `gyro` → `converted_gyro` → `pressure` → `quat` の順で届きます。

### 設定

| メソッド | CORE | INSOLE |
|---|---|---|
| `read_device_information()` / `print_device_information()` | ○ | ○ |
| `set_acc_range(2 / 4 / 8 / 16)` | ○ | ○ |
| `set_gyro_range(250 / 500 / 1000 / 2000)` | ○ | ○ |
| `set_led(is_on, pattern)` / `set_led_brightness(0-255)` / `set_lr(0 / 1)` | ○ | - |
| `set_mount_position(0-3)` | - | ○ |
| `set_data_streaming_mode(1 / 3 / 4)` | - | ○ |
| `read_pressure_calibrations()` | - | ○ |

範囲外の値は `ValueError`、接続中のデバイスで使えないメソッドは `UnsupportedOperationError` になります。

INSOLE の streaming mode は 1 = quat・gyro・acc（200Hz）、3 = gyro・acc・圧力（200Hz）、4 = quat・gyro・acc・圧力（100Hz）です。

### FIFO（ロスレス収録）

```python
from orphe_core_insole import FifoRecorder

fifo = FifoRecorder(device)                     # connect() 済みの OrpheCoreInsole
fifo.on('samples', lambda samples: ...)         # 回収したサンプル（FifoSample のリスト、古い順）
fifo.on('data_loss', lambda info: print(info))  # 回復できない欠損が確定したとき
if await fifo.start():                          # 未対応の FW などでは False（error イベントで理由）
    await asyncio.sleep(60)
    await fifo.stop()                           # 取りこぼしを回収してからリアルタイム通信に戻す
    fifo.save_csv('capture.csv')
    print(fifo.collected_count, fifo.dropped_count)  # dropped_count が 0 なら欠損なし
```

### 歩行解析（INSOLE）

```python
from orphe_core_insole import InsoleGait

gait = InsoleGait(device)                        # connect() 済みの OrpheCoreInsole（INSOLE）
gait.on('gait', lambda row: print(row))          # 1 歩ぶんがそろったとき（GaitRow）
gait.on('step_loss', lambda info: print(info))   # 歩の欠けが決まったとき
if await gait.start():
    await asyncio.sleep(60)
    await gait.stop()
    gait.save_csv('gait.csv')
```

### 時刻合わせ

```python
device = OrpheCoreInsole(time_sync_samples=5)   # 往復時間を測る回数（0 で合わせない）
await device.connect()
print(device.time_sync)                         # SyncTimeResult（往復時間と書き込んだ時刻）
await device.sync_device_time()                 # あとから合わせ直す
print((await device.read_date_time()).date)     # デバイスの時刻
```

### 圧力の換算（INSOLE）

```python
calibrations = await device.read_pressure_calibrations()  # 非対応 FW なら None
device.on('converted_pressure', lambda pressure: print(pressure.values))  # [N]
await device.start_sensor_values_notification()
```

### スキャン

```python
for result in await OrpheCoreInsole().scan():   # all_devices=True で ORPHE 以外も
    print(result.name, result.address, result.rssi, result.kind)
```

## サンプル

`examples/` にあります。CORE / INSOLE どちらでも動くものは自動判別します。

| ファイル | 内容 |
|---|---|
| `scan.py` | 周りの BLE デバイスと種別を表示 |
| `device_information.py` | device information を表示・設定 |
| `get_sensor_values.py` | センサー値を表示 |
| `get_step_analysis.py` | 歩行解析を表示（CORE） |
| `get_insole_gait.py` | 歩行解析を 1 歩ずつ表示して CSV に保存（INSOLE） |
| `blink_led.py` | LED を点滅（CORE） |
| `multi_devices.py` | 2 台同時接続 |
| `osc.py` | 加速度を OSC で送信（`python-osc`） |
| `plot_sensor_values.py` | 加速度・圧力をリアルタイム描画（`matplotlib`）。`pressure` 引数で圧力 |
| `fifo_record.py` | FIFO で収録して CSV に保存 |
| `gui.py` | tkinter で 2 台の接続・切断 |

```bash
python examples/get_sensor_values.py
```
