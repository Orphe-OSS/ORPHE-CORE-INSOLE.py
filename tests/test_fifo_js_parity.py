"""
JS 版（ORPHE Core-Insole.js）の FIFO 実装との一致確認

JS 版のリポジトリ（orphe-device）がこのリポジトリと同じ階層にあり、node 22 以上があるときだけ動く
（場所は ORPHE_CORE_INSOLE_JS_DIR で変えられる）。同じ入力を JS 版の関数に通し、
INSOLE の 0x36 パケットの CSV・デコード結果と、ループ状態の遷移が一致することを確かめる。
CORE（8×12B）は JS 版が対応していないので比べない。
"""

from __future__ import annotations

import json
import math
import os
import random
import shutil
import subprocess
import unittest
from pathlib import Path

from orphe_core_insole.fifo.protocol import INSOLE_LAYOUTS, FifoScale, decode_fifo_packet, raw_store_to_csv
from orphe_core_insole.fifo.state import FifoLoopState
from orphe_core_insole.pressure import (
    PressureCalibration,
    decode_pressure_calibration,
    legacy_pressure_to_newton,
    pressure_to_newton,
)

JS_DIR = Path(os.environ.get('ORPHE_CORE_INSOLE_JS_DIR', Path(__file__).resolve().parents[2] / 'orphe-device'))
NODE = shutil.which('node')

JS_SCRIPT = r"""
import { readFileSync } from 'node:fs';
const dir = process.argv[1];
const { rawStoreToCSV, decodeFifoPacket } = await import(dir + '/src/fifo/protocol.ts');
const { FifoLoopState } = await import(dir + '/src/fifo/state.ts');
const calib = await import(dir + '/src/protocol/pressure-calibration.ts');
const input = JSON.parse(readFileSync(0, 'utf8'));

const toView = (hex) => new DataView(Uint8Array.from(Buffer.from(hex, 'hex')).buffer);
const store = new Map(input.packets.map(([serial, hex]) => [serial, toView(hex)]));
const decoded = input.packets.map(([, hex]) => decodeFifoPacket(toView(hex)).samples.map((s) => [
  s.serial_number, s.packet_number, s.t,
  [s.converted_gyro.x, s.converted_gyro.y, s.converted_gyro.z],
  [s.converted_acc.x, s.converted_acc.y, s.converted_acc.z],
  s.press.values,
]));

const state = new FifoLoopState();
const trace = [];
for (const op of input.ops) {
  if (op[0] === 'range') trace.push(state.calcRequestRange(op[1], op[2], op[3]));
  else trace.push(state.updateAfterResponse(new Set(op[1]), new Set(op[2]), op[3], op[4]));
  trace.push([state.lastSerial, state.carryOver.map((pair) => [...pair]), state.dropped, state.resyncPending,
              state.lossEvents.map((e) => [e.reason, e.dropped])]);
}
const calibrations = input.calibrations.map((c) => (c ? { func: c[0], coefficients: c[1] } : null));
const decodedCalibrations = input.calibrationPackets.map((hex) => {
  const r = calib.decodePressureCalibration(toView(hex));
  return r ? [r.sensorIndex, r.calibration.func, r.calibration.coefficients] : null;
});
const newton = input.adc.map((raw, i) => [
  calib.legacyPressureToNewton(raw),
  calib.pressureToNewton(raw, calibrations[i % 6]),
]);
process.stdout.write(JSON.stringify({
  csv: rawStoreToCSV(store), csvCalibrated: rawStoreToCSV(store, calibrations),
  decoded, trace, decodedCalibrations, newton,
}));
"""


def random_packet(rng: random.Random, serial: int) -> bytes:
    data = bytearray(rng.randrange(256) for _ in range(104))
    data[0] = 0x36
    data[1:3] = serial.to_bytes(2, 'big')
    data[3:6] = bytes([rng.randrange(24), rng.randrange(60), rng.randrange(60)])
    data[6:8] = rng.randrange(1000).to_bytes(2, 'big')
    if rng.random() < 0.2:
        data[8:14] = bytes(6)  # 0 と -0 の表記を確かめる
    for frame in range(4):
        for ch in range(6):
            o = 8 + frame * 24 + 12 + 2 * ch
            data[o] &= 0x0F  # 圧力の ADC は 12 ビット（巨大な値では exp の最下位ビットが JS と違いうる）
    return bytes(data)


def random_ops(rng: random.Random) -> list[list]:
    ops: list[list] = []
    current = rng.randrange(65536)
    for _ in range(400):
        current = (current + rng.choice((0, 5, 40, 200, 1600))) % 65536
        ops.append(['range', current, rng.randrange(0, 1600), rng.choice((0, 50, 200))])
        start = rng.randrange(65536)
        size = rng.randrange(0, 60)
        serials = [(start + i) % 65536 for i in range(size)]
        ble_loss = rng.sample(serials, k=min(len(serials), rng.choice((0, 0, 3, 120))))
        no_data = rng.sample(serials, k=min(len(serials), rng.choice((0, 0, 0, 2))))
        ops.append(['update', ble_loss, no_data, start, size])
    return ops


def run_python_ops(ops: list[list]) -> list:
    state = FifoLoopState()
    trace: list = []
    for op in ops:
        if op[0] == 'range':
            trace.append(list(state.calc_request_range(op[1], op[2], op[3])))
        else:
            trace.append(state.update_after_response(set(op[1]), set(op[2]), op[3], op[4]))
        trace.append([state.last_serial, [list(pair) for pair in state.carry_over], state.dropped,
                      state.resync_pending, [[event.reason, event.dropped] for event in state.loss_events]])
    return trace


@unittest.skipUnless(NODE and (JS_DIR / 'src/fifo/protocol.ts').exists(), 'JS 版か node が見つからない')
class TestFifoJsParity(unittest.TestCase):

    def run_js(self, packets: list[tuple[int, bytes]], ops: list[list], extra: dict) -> dict:
        payload = json.dumps({'packets': [[serial, data.hex()] for serial, data in packets], 'ops': ops, **extra})
        result = subprocess.run(
            [NODE, '--experimental-strip-types', '--no-warnings', '--input-type=module', '-e', JS_SCRIPT,
             str(JS_DIR)],
            input=payload, capture_output=True, text=True, check=True, timeout=60)
        return json.loads(result.stdout)

    def test_same_as_js(self):
        rng = random.Random(7)
        serials = rng.sample(range(65536), 300)
        packets = [(serial, random_packet(rng, serial)) for serial in serials]
        ops = random_ops(rng)
        # 校正係数: 多項式・指数式・未書き込み（全部 1.0）・無し。デコードは壊れた長さも混ぜる
        calibrations = [[1, [6e-11, -2e-7, 3e-4, -0.16, 22.5]], [0, [2.5, 0.0021, 4.1, 0, 0]],
                        [0, [1.0, 1.0, 1.0, 1.0, 1.0]], None, [1, [1e-12, 0, 0, 0.02, -3.0]],
                        [0, [-1.0, 0.001, 0.5, 0, 0]]]
        calibration_packets = [bytes([0x39, rng.randrange(6), rng.randrange(3)])
                               + bytes(rng.randrange(256) for _ in range(40 if rng.random() < 0.9 else 30))
                               for _ in range(200)]
        adc = [rng.randrange(4096) for _ in range(600)]
        js = self.run_js(packets, ops, {'calibrations': calibrations, 'adc': adc,
                                        'calibrationPackets': [p.hex() for p in calibration_packets]})

        # ±16G / ±2000dps（JS 版の固定レンジ）で比べる
        scale = FifoScale()
        self.assertEqual(raw_store_to_csv(dict(packets), 'insole', scale), js['csv'])

        decoded = [[[s.serial_number, s.packet_number, s.t, list(s.gyro), list(s.acc), list(s.pressure)]
                    for s in decode_fifo_packet(data, INSOLE_LAYOUTS[0x36], scale)] for _, data in packets]
        self.assertEqual(decoded, js['decoded'])

        self.assertEqual(run_python_ops(ops), js['trace'])

        # 個体別圧力校正係数
        py_calibrations = [PressureCalibration(c[0], tuple(float(v) for v in c[1])) if c else None
                           for c in calibrations]
        self.assertEqual(raw_store_to_csv(dict(packets), 'insole', scale, py_calibrations), js['csvCalibrated'])
        def finite(value):  # JSON では NaN・無限大は null、大きな整数値は int になるのでそろえる
            return float(value) if value is not None and math.isfinite(value) else None

        decoded_calibrations = []
        for packet in calibration_packets:
            result = decode_pressure_calibration(packet)
            decoded_calibrations.append(
                [result[0], result[1].func, [finite(v) for v in result[1].coefficients]] if result else None)
        js_calibrations = [[c[0], c[1], [finite(v) for v in c[2]]] if c else None for c in js['decodedCalibrations']]
        self.assertEqual(decoded_calibrations, js_calibrations)
        newton = [[legacy_pressure_to_newton(raw), pressure_to_newton(raw, py_calibrations[i % 6])]
                  for i, raw in enumerate(adc)]
        # exp の最下位ビットは Python（OS の数学ライブラリ）と JS（V8）で違いうるので、相対 1e-12 で比べる
        for (py_legacy, py_calibrated), (js_legacy, js_calibrated) in zip(newton, js['newton'], strict=True):
            self.assertTrue(math.isclose(py_legacy, js_legacy, rel_tol=1e-12, abs_tol=0))
            self.assertTrue(math.isclose(py_calibrated, js_calibrated, rel_tol=1e-12, abs_tol=0))


if __name__ == '__main__':
    unittest.main()
