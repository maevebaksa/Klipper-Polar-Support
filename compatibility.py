"""Reject untested motion interfaces and stale standalone binaries."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def fingerprint(klipper):
    expected = json.loads((ROOT / 'supported-interfaces.json').read_text())
    for name, checksum in expected.items():
        path = klipper / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != checksum:
            raise RuntimeError('Unsupported Klipper motion interface: ' + name
                               + '. Update Polar support to a compatible version.')
    parts = [ROOT / 'supported-interfaces.json', ROOT / 'csrc/polar_center.c',
             ROOT / 'klippy/kinematics/polar_center.py',
             ROOT / 'klippy/kinematics/polar_native_arc.py']
    return hashlib.sha256(b''.join(p.read_bytes() for p in parts)).hexdigest()


def check_installed(klipper):
    current = fingerprint(klipper)
    record = ROOT / 'build/fingerprint'
    if not record.is_file() or record.read_text().strip() != current:
        raise RuntimeError('Polar solver needs installation: run bash '
                           + str(ROOT / 'install.sh') + ' while idle.')
