"""orphe-core-insole コマンド（実機の代わりに SimulatedClient を使う）"""

import csv
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
from pathlib import Path
from unittest import mock

from fifo_sim import FifoClient
from helpers import SimulatedClient, fake_discover

from orphe_core_insole import OrpheCoreInsole, cli
from orphe_core_insole.profiles import AutoProfile, CoreProfile, InsoleProfile

DEVICES = (("INS-L TEST", "BB"), ("CR-3 TEST", "CC"), ("Keyboard", "AA"))


class CliCase(unittest.TestCase):

    def setUp(self) -> None:
        self.clients: list[SimulatedClient] = []
        self.apply_writes = True
        self.serial_start = 0
        # FIFO に対応した FW を模擬するときの FifoClient のオプション
        self.fifo_options: dict | None = None
        # SimulatedClient に渡す追加のオプション（FW のリリース日・校正係数）
        self.sim_options: dict = {}

    def create_device(self, args):
        core, insole = CoreProfile(), InsoleProfile()
        core.write_wait_sec = insole.write_wait_sec = 0
        profile = {'core': core, 'insole': insole}.get(args.device) or AutoProfile(core, insole)

        def factory(ble_device):
            kind = 'insole' if ble_device.name.startswith('INS') else 'core'
            if self.fifo_options is not None:
                client = FifoClient(ble_device, kind=kind, **self.fifo_options)
            else:
                client = SimulatedClient(ble_device, kind, apply_writes=self.apply_writes,
                                         serial_start=self.serial_start, **self.sim_options)
            self.clients.append(client)
            return client

        return OrpheCoreInsole(profile, verbose=False, discover=fake_discover(*DEVICES),
                               client_factory=factory, monitor_interval_sec=0.02)

    def run_cli(self, *argv: str) -> tuple[int, str, str]:
        out, err = StringIO(), StringIO()
        with mock.patch.object(cli, 'create_device', self.create_device), \
                redirect_stdout(out), redirect_stderr(err):
            code = cli.main(list(argv))
        return code, out.getvalue(), err.getvalue()


class TestScanInfoSet(CliCase):

    def test_scan(self):
        code, out, _ = self.run_cli('scan')
        self.assertEqual(code, 0)
        self.assertIn('insole', out)
        self.assertIn('CR-3 TEST', out)
        self.assertNotIn('Keyboard', out)
        _, out, _ = self.run_cli('scan', '--all')
        self.assertIn('Keyboard', out)

    def test_info(self):
        code, out, err = self.run_cli('info', '--device', 'core')
        self.assertEqual(code, 0)
        self.assertIn('Kind: core', out)
        self.assertIn('LED: 200', out)
        self.assertIn('切断しました', err)
        self.assertFalse(self.clients[0].is_connected)

    def test_set(self):
        code, out, _ = self.run_cli('set', '--acc', '16', '--gyro', '250', '--mount', '3', '--mode', '3')
        self.assertEqual(code, 0)
        client = self.clients[0]
        self.assertEqual((client.state[1], client.state[8], client.state[9], client.mode), (3, 3, 0, 3))
        self.assertIn('Mount Position: 3', out)

    def test_set_rejects_core_only_option_on_insole(self):
        code, _, err = self.run_cli('set', '--led', '1', '0')
        self.assertEqual(code, 2)
        self.assertIn('only available for ORPHE CORE', err)

    def test_set_requires_option(self):
        code, _, err = self.run_cli('set')
        self.assertEqual(code, 2)
        self.assertEqual(self.clients, [])
        self.assertIn('設定する項目', err)


class TestPressureCalibration(CliCase):

    CALIBRATIONS = ((1, (0.0, 0.0, 0.0, 0.01, 1.0)), None, None, None, None, (0, (2.0, 0.001, 3.0, 0.0, 0.0)))

    def test_info_shows_calibrations(self):
        self.sim_options = {'release_date': 20260428, 'calibrations': self.CALIBRATIONS}
        code, out, _ = self.run_cli('info')
        self.assertEqual(code, 0)
        self.assertIn('ch1: func=1 (0, 0, 0, 0.01, 1)', out)
        self.assertIn('ch2: なし（固定式で換算）', out)
        self.assertIn('ch6: func=0 (2, 0.001, 3, 0, 0)', out)

    def test_info_old_firmware(self):
        self.sim_options = {'release_date': 20260101, 'calibrations': self.CALIBRATIONS}
        _, out, _ = self.run_cli('info')
        self.assertIn('非対応 FW', out)

    def test_stream_csv_has_newton(self):
        self.sim_options = {'release_date': 20260428, 'calibrations': self.CALIBRATIONS}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'out.csv'
            code, _, err = self.run_cli('stream', '--duration', '0.2', '--csv', str(path))
            with path.open() as f:
                row = next(csv.DictReader(f))
        self.assertEqual(code, 0, err)
        self.assertIn('圧力校正係数: 2/6 ch', err)
        self.assertEqual(row['pressure_1'], '100')
        self.assertAlmostEqual(float(row['pressure_1[N]']), 0.01 * 100 + 1.0)  # ch1 は係数（多項式）
        self.assertEqual(float(row['pressure_2[N]']), 0.0)                     # 固定式: 100 は 240mV 以下


class TestStream(CliCase):

    def test_stream_writes_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'out.csv'
            code, _, err = self.run_cli('stream', '--duration', '0.3', '--mode', '3', '--csv', str(path))
            self.assertEqual(code, 0)
            with path.open() as f:
                rows = list(csv.DictReader(f))
        self.assertGreater(len(rows), 10)
        self.assertEqual(rows[0]['pressure_1'], '100')
        self.assertEqual(rows[0]['quat_w'], '')  # mode 3 はクォータニオンなし
        self.assertAlmostEqual(float(rows[0]['acc_z']), 1.0, places=3)
        self.assertIn('欠損 0 packets', err)

    def test_loss_ignores_first_packet_and_wraps(self):
        # 接続直後のシリアル番号が大きくても欠損にしない。65535 → 0 も連続とみなす
        self.serial_start = 65500
        code, _, err = self.run_cli('stream', '--device', 'core', '--duration', '1.2')
        self.assertEqual(code, 0)
        self.assertGreater(self.clients[0].serial, 0)
        self.assertLess(self.clients[0].serial, 65500)
        self.assertNotRegex(err, r"lost [1-9]")
        self.assertIn('欠損 0 packets', err)

    def test_loss_is_counted_from_serial_gaps(self):
        from orphe_core_insole.cli import StreamStats
        from orphe_core_insole.packets import SensorValuesPacket
        stats = StreamStats()
        for serial in (144, 145, 148, 65535, 1):
            stats.add_packet(SensorValuesPacket(50, serial, 0))
        # 145→148 で 2、148→65535 で 65386、65535→1 で 1
        self.assertEqual(stats.lost_packets, 2 + (65535 - 148 - 1) + 1)

    def test_stream_print(self):
        code, out, _ = self.run_cli('stream', '--device', 'core', '--duration', '0.1', '--print')
        self.assertEqual(code, 0)
        self.assertIn('quat=(1.0000', out)

    def test_mode_is_insole_only(self):
        code, _, err = self.run_cli('stream', '--device', 'core', '--mode', '1', '--duration', '0.1')
        self.assertEqual(code, 2)
        self.assertIn('INSOLE だけ', err)


class TestStep(CliCase):

    def test_insole_step_prints_rows_and_saves_csv(self):
        self.sim_options = {'steps': 3}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'gait.csv'
            code, out, err = self.run_cli('step', '--device', 'insole', '--duration', '0.3', '--csv', str(path))
            rows = list(csv.reader(path.open()))
        self.assertEqual(code, 0, err)
        self.assertEqual(out.count('[gait] step_number='), 3)
        self.assertIn('合計 3 歩', err)
        self.assertEqual(rows[0][:3], ['step_number', 'gait_type', 'stride_direction'])
        self.assertEqual([row[:3] for row in rows[1:]], [[str(i), 'walk', 'forward'] for i in (1, 2, 3)])

    def test_csv_is_insole_only(self):
        code, _, err = self.run_cli('step', '--device', 'core', '--duration', '0.1', '--csv', 'x.csv')
        self.assertEqual(code, 2)
        self.assertIn('INSOLE だけ', err)


class TestFifo(CliCase):

    def test_insole_csv(self):
        self.fifo_options = {'release_date': 20260901}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'fifo.csv'
            code, _, err = self.run_cli('fifo', '--duration', '0.6', '--drain-timeout', '0.5', '--csv', str(path))
            lines = path.read_text().splitlines()
        self.assertEqual(code, 0, err)
        self.assertIn('FW: INS-TEST-1-0003-20260901-07', err)
        self.assertIn('欠損 0', err)
        self.assertTrue(lines[0].startswith('serial_number,timestamp,gyro_x[dps]'))
        self.assertIn('press6[N]', lines[0])
        self.assertGreater(len(lines), 40)
        # FIFO のあとは streaming mode 4 に戻す
        self.assertEqual(self.clients[0].mode_writes[-1], 4)

    def test_core_csv_has_no_pressure(self):
        self.fifo_options = {'release_date': 20260905}
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'fifo.csv'
            code, _, err = self.run_cli('fifo', '--device', 'core', '--duration', '0.4', '--csv', str(path))
            header = path.read_text().splitlines()[0]
        self.assertEqual(code, 0, err)
        self.assertNotIn('press', header)
        self.assertEqual(self.clients[0].mode_writes[-1], 1)

    def test_old_firmware(self):
        self.fifo_options = {'release_date': 20260101}
        code, _, err = self.run_cli('fifo', '--device', 'core', '--duration', '0.2')
        self.assertEqual(code, 1)
        self.assertIn('FIFO を開始できませんでした', err)
        self.assertIn('20260905', err)


class TestCheck(CliCase):

    def assert_all_pass(self, out: str) -> None:
        failed = [line for line in out.splitlines() if line.startswith(('[FAIL]', '[WARN]'))]
        self.assertEqual(failed, [])

    def test_insole(self):
        code, out, err = self.run_cli('check', '--duration', '0.4', '--write')
        self.assertEqual(code, 0, out)
        self.assert_all_pass(out)
        for mode in (1, 3, 4):
            self.assertIn(f'[PASS] mode {mode} の受信レート', out)
        self.assertIn('[PASS] mode 4 のクォータニオン', out)
        self.assertIn('[PASS] mode 3 の圧力', out)
        self.assertIn('[PASS] マウント位置の書き込み', out)
        self.assertIn('[PASS] 切断', out)
        self.assertIn('FAIL 0', err)
        # 設定は元に戻っている
        self.assertEqual(self.clients[0].state[1], 1)
        self.assertEqual(self.clients[0].mode, 4)

    def test_fifo_is_skipped_without_firmware_info(self):
        code, out, _ = self.run_cli('check', '--duration', '0.2')
        self.assertEqual(code, 0, out)
        self.assertIn('[INFO] FIFO: FW 情報が読めない', out)

    def test_fifo(self):
        self.fifo_options = {'release_date': 20260901}
        code, out, _ = self.run_cli('check', '--duration', '0.5')
        self.assertEqual(code, 0, out)
        self.assert_all_pass(out)
        self.assertIn('[PASS] FIFO の回収', out)
        self.assertIn('[PASS] FIFO のサンプリングレート', out)
        self.assertIn('[PASS] FIFO の圧力', out)
        self.assertIn('[PASS] 時刻合わせ', out)
        self.assertIn('[PASS] 歩行解析の購読', out)

    def test_core_fifo(self):
        # CORE は 8 サンプル × 12B。レンジは device information（±8G）で換算する
        self.fifo_options = {'release_date': 20260905}
        code, out, _ = self.run_cli('check', '--device', 'core', '--duration', '0.5')
        self.assertEqual(code, 0, out)
        self.assert_all_pass(out)
        self.assertIn('[PASS] FIFO のサンプリングレート', out)
        self.assertIn('[PASS] FIFO の静止時の加速度', out)

    def test_core(self):
        code, out, _ = self.run_cli('check', '--device', 'core', '--duration', '0.4', '--write')
        self.assertEqual(code, 0, out)
        self.assert_all_pass(out)
        self.assertIn('[PASS] SENSOR_VALUES の受信レート', out)
        self.assertIn('[PASS] LED の明るさの書き込み', out)
        self.assertIn('[PASS] 歩行解析の購読', out)

    def test_fails_when_settings_are_not_applied(self):
        self.apply_writes = False
        code, out, _ = self.run_cli('check', '--device', 'core', '--duration', '0.2', '--write')
        self.assertEqual(code, 1)
        self.assertIn('[FAIL] 加速度レンジの書き込み', out)

    def test_settings_are_not_written_without_write_option(self):
        for device in ('core', 'insole'):
            with self.subTest(device=device):
                self.clients = []
                code, out, _ = self.run_cli('check', '--device', device, '--duration', '0.2')
                self.assertEqual(code, 0, out)
                self.assertIn('[INFO] 設定の書き込み: 確かめていません', out)
                self.assertNotIn('書き込み: 16', out)
                # device information への書き込み（CORE 0x01 / INSOLE 0x09）も LED のコマンドもない
                commands = {data[0] for _, data in self.clients[0].writes}
                self.assertFalse(commands & {0x01, 0x02, 0x09}, commands)

    def test_restores_setting_when_interrupted(self):
        client_reads = []

        class Interrupted(Exception):
            pass

        original = OrpheCoreInsole.read_device_information

        async def read_device_information(device):
            client_reads.append(None)
            if len(client_reads) == 3:  # 書き込んだあとの読み直しで止まる
                raise Interrupted
            return await original(device)

        with mock.patch.object(OrpheCoreInsole, 'read_device_information', read_device_information):
            code, _, _ = self.run_cli('check', '--device', 'core', '--duration', '0.2', '--write')
        self.assertEqual(code, 1)
        client = self.clients[0]
        self.assertEqual(client.state[8], 2)  # 加速度レンジは元の ±8G（index 2）に戻っている

    def test_not_found(self):
        global DEVICES
        saved, DEVICES = DEVICES, (("Keyboard", "AA"),)
        try:
            code, out, _ = self.run_cli('check')
        finally:
            DEVICES = saved
        self.assertEqual(code, 1)
        self.assertIn('[FAIL] 接続', out)


if __name__ == "__main__":
    unittest.main()
