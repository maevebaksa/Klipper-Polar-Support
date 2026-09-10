#!/usr/bin/env python3
"""Compare exact-stop and corner-limited planning using real Klippy/C queues.

Optional --gcode accepts this project's Orca Benchy export and replays layer 0.
Without it, generate curved walls, a sharp reversal and exact center crossings.
No printer is contacted. Only motion/extrusion commands are replayed.
"""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--klipper', type=Path, required=True)
    parser.add_argument('--dictionary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gcode', type=Path)
    args = parser.parse_args()
    root, out = args.klipper.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    config = (Path(__file__).parents[1]/'config/polar3d-printrboard-center.cfg').read_text()
    config = re.sub(r'^#.*\n', '', config, flags=re.M)
    pins = iter(range(1, 30))
    config = re.sub(r'(?m)^((?:\w+_pin|pin):\s*)[!^]*P[A-F]\d+',
                    lambda m: m[1]+str(next(pins)), config)
    (out/'temperature').write_text('25000\n')
    config = config.replace('sensor_type: EPCOS 100K B57560G104F', 'sensor_type: temperature_host')
    config = re.sub(r'(?m)^sensor_pin:.*$', 'sensor_path: '+str(out/'temperature'), config)
    for key, value in dict(min_extrude_temp=0, max_velocity=50, max_accel=1500,
                           max_radial_velocity=50, max_radial_accel=1500,
                           max_z_velocity=5, max_z_accel=100, max_angular_accel=100).items():
        config = re.sub(r'(?m)^'+key+r':.*$', key+': '+str(value), config)
    config += '\n[polar_center_audit]\n[polar_benchmark]\n'
    commands = ['G28', 'G90', 'M83', 'G1 Z5 F120', 'G1 X20 Y0 F1200', 'POLAR_BENCH_START']
    expected_e = 0.
    if args.gcode:
        active = False
        for line in args.gcode.read_text().splitlines():
            if line.startswith('M117 Printing Layer 1/'):
                break
            if line.startswith('M117 Printing Layer 0/'):
                active = True
            code = line.split(';')[0].strip()
            if active and code and code.split()[0] in ('G0','G1','G92','SET_VELOCITY_LIMIT'):
                commands.append(code)
                if code.split()[0] in ('G0','G1'):
                    e = re.search(r'\bE([-+]?\d*\.?\d+)', code)
                    if e:
                        expected_e += float(e[1])
        if not active:
            raise SystemExit('Expected an Orca export with M117 Printing Layer 0/... marker')
    else:
        for i in range(1, 721):
            angle = i*2*math.pi/360
            commands.append('G1 X%.9f Y%.9f E.01 F1800' % (20*math.cos(angle),20*math.sin(angle)))
            expected_e += .01
        commands += ['G1 X30 Y0 E.1', 'G1 X20 Y0 E.1',
                     'G1 X-20 Y0 E.1', 'G1 X20 Y0 E.1']
        expected_e += .4
    commands += ['POLAR_BENCH_END E=%.10f' % expected_e, 'M400']
    (out/'motion.gcode').write_text('\n'.join(commands)+'\n')
    audit = root/'klippy/extras/polar_center_audit.py'
    bench = root/'klippy/extras/polar_benchmark.py'
    old = {p: p.read_bytes() if p.exists() else None for p in (audit,bench)}
    audit.write_bytes(Path(__file__).with_name('polar_center_audit.py').read_bytes())
    bench.write_text('''import json, logging
class Benchmark:
    def __init__(self, config):
        self.printer = config.get_printer()
        g = self.printer.lookup_object('gcode')
        g.register_command('POLAR_BENCH_START', self.start)
        g.register_command('POLAR_BENCH_END', self.end)
    def start(self, g):
        self.th = self.printer.lookup_object('toolhead')
        self.th.flush_step_generation()
        self.start_time = self.th.get_last_move_time()
        self.es = self.th.get_extruder().extruder_stepper.stepper
        self.steps = self.es.get_mcu_position()
    def end(self, g):
        self.th.flush_step_generation()
        elapsed = self.th.get_last_move_time()-self.start_time
        fed = (self.es.get_mcu_position()-self.steps)*self.es.get_step_dist()
        expected = g.get_float('E')
        if abs(fed-expected) > self.es.get_step_dist():
            raise g.error('Benchmark extrusion mismatch')
        audit = self.printer.lookup_object('polar_center_audit')
        logging.info('POLAR_BENCHMARK '+json.dumps(dict(seconds=elapsed,
            e_mm=fed, expected_e_mm=expected, junctions=audit.junction_checks,
            moving_junctions=audit.moving_junctions, trajectory_samples=audit.trajectory_checks)))
def load_config(config):
    return Benchmark(config)
''')
    results = {}
    try:
        for name, scv in [('exact_stop',0),('polar_corners',5)]:
            cfg = out/(name+'.cfg')
            cfg.write_text(re.sub(r'(?m)^square_corner_velocity:.*$',
                                  'square_corner_velocity: '+str(scv), config))
            log = out/(name+'.log')
            log.unlink(missing_ok=True)
            process = subprocess.run([sys.executable,str(root/'klippy/klippy.py'),str(cfg),
                '-i',str(out/'motion.gcode'),'-o',str(out/(name+'.mcu')),
                '-d',str(args.dictionary.resolve()),'-l',str(log)],
                capture_output=True,text=True,timeout=120)
            matches = re.findall(r'^POLAR_BENCHMARK (.*)$',log.read_text(),re.M)
            if process.returncode or not matches:
                raise RuntimeError(process.stderr+'\n'+log.read_text()[-5000:])
            results[name] = json.loads(matches[-1])
    finally:
        for path,data in old.items():
            if data is None:
                path.unlink()
            else:
                path.write_bytes(data)
    results['time_reduction_percent'] = 100*(1-results['polar_corners']['seconds']/results['exact_stop']['seconds'])
    assert results['polar_corners']['moving_junctions'] > results['exact_stop']['moving_junctions']
    assert results['time_reduction_percent'] > 0
    assert abs(results['polar_corners']['e_mm']-results['exact_stop']['e_mm']) < .02
    (out/'result.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results,indent=2))


if __name__ == '__main__':
    main()
