#!/usr/bin/env python3
"""Real Klippy + compiled C step generation in MCU file-output mode.

Use a hostsimulator dictionary built by Klipper. No printer is contacted.
"""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--klipper', required=True, type=Path)
    parser.add_argument('--dictionary', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--features', action='store_true', help='Enable retraction and native arcs')
    args = parser.parse_args()
    root, out = args.klipper.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    sample = Path(__file__).parents[1]/'config/polar3d-printrboard-center.cfg'
    config = re.sub(r'^#.*\n', '', sample.read_text(), flags=re.M)
    pins = iter(range(1, 30))
    config = re.sub(r'(?m)^((?:\w+_pin|pin):\s*)[!^]*P[A-F]\d+',
                    lambda m: m[1]+str(next(pins)), config)
    config = config.replace('min_extrude_temp: 170', 'min_extrude_temp: 0')
    config = config.replace('sensor_type: EPCOS 100K B57560G104F',
                            'sensor_type: temperature_host')
    temp = out/'temperature'
    temp.write_text('25000\n')
    config = re.sub(r'(?m)^sensor_pin:.*$', 'sensor_path: '+str(temp), config)
    config += '\n[force_move]\nenable_force_move: True\n[polar_center_audit]\n[gcode_arcs]\n'
    # Deliberately test pressure advance as well as synchronized XYZ/E.
    config = config.replace('[extruder]', '[extruder]\npressure_advance: 0.04')
    if args.features:
        config = config.replace('center_retract_length: 0\n', 'center_retract_length: 0.3\n')
        config = config.replace('polar_native_arcs: False', 'polar_native_arcs: True')
        config += '\n[firmware_retraction]\nretract_length: 0.4\n'
    cfg = out/'simulation.cfg'
    cfg.write_text(config)
    commands = ['G28', 'G90', 'M82', 'G1 Z10 F180', 'G1 X30 Y0 F600',
                'POLAR_TEST_ANCHOR', 'POLAR_TEST_POSITION X=30 Y=0 Z=10 E=0']
    e = 0.
    expected_endpoints = 1
    def move(x, y, z=10., extrude=0., speed=2400):
        nonlocal e, expected_endpoints
        e += extrude
        commands.append('G1 X%.9f Y%.9f Z%.9f E%.9f F%d' % (x,y,z,e,speed))
        commands.append('POLAR_TEST_POSITION X=%.9f Y=%.9f Z=%.9f E=%.9f' % (x,y,z,e))
        expected_endpoints += 1
    # Exact crossings and separate arrival / dwell / departure in all quadrants.
    for angle in [0, .3, math.pi/2, math.pi, -math.pi/2, 2.9]:
        x, y = 30*math.cos(angle), 30*math.sin(angle)
        move(x, y)
        move(-x, -y, 11., .5)
        move(0, 0, 11., .1)
        commands += ['G4 P100', 'G1 Z12 F180', 'G1 E%.9f F60' % (e-.5),
                     'G1 E%.9f F60' % e]
        move(y, -x, 12., .1)
    # Repeated reversals detect accumulated rounding errors in 14:3 gearing.
    move(30, 0)
    for i in range(120):
        move(-30 if i%2==0 else 30, 0, 10., .02)
    # Near-center paths preserve the offset and stay finite at both high/low F.
    for offset in [1.e-6, .001, .01, .1, 1., 5.]:
        move(-30, offset)
        move(30, offset, 11., .5, 300)
        move(-30, -offset, 10., .5)
    # Multiple full rotations, reversals, and atan2 branch-cut transitions.
    for i in range(65):
        a = 4*math.pi*i/64
        move(20*math.cos(a), 20*math.sin(a), extrude=.01)
    # Queued moves without an audit flush at each junction, G2/G3, and
    # relative coordinates/extrusion all use the native toolhead path.
    commands += ['G1 X20 Y0 F1200', 'G1 X0 Y20', 'G1 X-20 Y0',
                 'G1 X20 Y0', 'G1 X0 Y0', 'G1 X0 Y-20', 'G1 X0 Y0',
                 'G1 X20 Y0', 'G3 X0 Y0 I-10 J0 F600',
                 'G2 X20 Y0 I10 J0 F600', 'G91', 'M83',
                 'G1 X-40 Y0 E0.1 F600', 'G90', 'M82']
    e += .1
    move(30, 0)
    # Invalid endpoints and extrusion must not queue the first half of a line.
    move(30, 0)
    commands += ['POLAR_TEST_REJECT X=-101', 'POLAR_TEST_REJECT X=-30 Z=151',
                 'POLAR_TEST_REJECT X=-30 E=10000']
    if args.features:
        commands += ['M83', 'M221 S200', 'M220 S50', 'G92 E0']
        for i in range(24):
            commands.append(('G2' if i % 2 else 'G3') + ' X30 Y0 I-30 J0 E0.05 F1200')
            e += .1
            commands.append('POLAR_TEST_POSITION X=30 Y=0 E=%.9f' % e)
            expected_endpoints += 1
        commands += ['M221 S100', 'M220 S100', 'M82', 'G92 E%.9f' % e]
        for cx, cy in [(20,20), (-20,20), (-20,-20), (20,-20)]:
            move(cx+5, cy)
            for cmd in ['G2', 'G3']:
                e += .1
                commands.append('%s X%g Y%g I-5 J0 E%.9f F1200' % (cmd,cx+5,cy,e))
                commands.append('POLAR_TEST_POSITION X=%g Y=%g E=%.9f' % (cx+5,cy,e))
                expected_endpoints += 1
        # Partial arcs change endpoints; test Cartesian queue restoration and
        # nonzero G92 XY offsets as well as absolute extrusion.
        move(30, 0)
        commands += ['G92 X130 Y100']
        for cmd, points in [
            ('G3', [(0,30), (-30,0), (0,-30), (30,0)]),
            ('G2', [(0,-30), (-30,0), (0,30), (30,0)])]:
            px, py = 30, 0
            for x, y in points:
                e += .05
                commands.append('%s X%g Y%g I%g J%g E%.9f F1200'
                                % (cmd,x+100,y+100,-px,-py,e))
                commands.append('POLAR_TEST_POSITION X=%g Y=%g E=%.9f' % (x,y,e))
                expected_endpoints += 1
                px, py = x, y
        commands += ['G92 X30 Y0']
        # Origin intersection and helical arcs retain the segmented path.
        move(20, 0)
        commands += ['G3 X20 Y0 I-10 J0 F600', 'G3 X-20 Y0 Z11 I-20 J0 F600']
        commands += ['POLAR_TEST_FEATURES', 'POLAR_TEST_ARC_REJECT']
        move(30, 0)
    commands += ['M18', 'POLAR_TEST_REJECT X=-30']
    # Homing after multiple rotations, then fresh coordinate-frame anchor.
    commands += ['G28', 'G1 Z10 F180', 'G1 X30 Y0 F600', 'POLAR_TEST_ANCHOR']
    move(-30, 0)
    commands += ['M400']
    gcode = out/'regression.gcode'
    gcode.write_text('\n'.join(commands)+'\n')
    audit = root/'klippy/extras/polar_center_audit.py'
    old = audit.read_bytes() if audit.exists() else None
    audit.write_bytes(Path(__file__).with_name('polar_center_audit.py').read_bytes())
    log = out/'klippy.log'
    log.unlink(missing_ok=True)
    try:
        process = subprocess.run([sys.executable, str(root/'klippy/klippy.py'), str(cfg),
                                  '-i', str(gcode), '-o', str(out/'mcu-output'),
                                  '-d', str(args.dictionary.resolve()), '-l', str(log)],
                                 capture_output=True, text=True, timeout=120)
    finally:
        if old is None:
            audit.unlink()
        else:
            audit.write_bytes(old)
    text = log.read_text()
    endpoints = text.count('POLAR_AUDIT endpoint PASS')
    rotations = text.count('POLAR_AUDIT rotation stationary XYZ/E PASS')
    rejected = text.count('POLAR_AUDIT rejected request atomicity PASS')
    samples = re.findall(r'POLAR_AUDIT trajectory samples=(\d+)', text)
    result = dict(exit_code=process.returncode, endpoint_checks=endpoints,
                  expected_endpoints=expected_endpoints, stationary_rotations=rotations,
                  rejected_move_checks=rejected, pressure_advance=.04, features_enabled=args.features,
                  native_arc_checks=text.count('POLAR_AUDIT native arc PASS'),
                  retraction_checks=text.count('POLAR_AUDIT retraction PASS'),
                  trajectory_samples=int(samples[-1]) if samples else 0,
                  backend='Real Klippy/C helpers, hostsimulator MCU file-output')
    result['passed'] = (process.returncode == 0 and endpoints == expected_endpoints
                        and rotations >= 120 and rejected == 4
                        and (not args.features or (result['native_arc_checks'] >= 40
                             and result['retraction_checks'] >= 120
                             and 'POLAR_AUDIT feature state PASS' in text
                             and 'POLAR_AUDIT native arc rejection atomicity PASS' in text)))
    (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
    if not result['passed']:
        print(process.stdout, process.stderr)
        print(text[-8000:])
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
