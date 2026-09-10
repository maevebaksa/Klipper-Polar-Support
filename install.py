#!/usr/bin/env python3
"""Install Polar support without modifying tracked Klipper source files."""
from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
from compatibility import fingerprint

ROOT = Path(__file__).resolve().parent
PLUGIN_FILE = "klippy/kinematics/polar_center.py"
LEGACY_FILES = ("klippy/chelper/kin_polar.c", "klippy/chelper/__init__.py")
LEGACY_INSTALLED = {
    "klippy/chelper/kin_polar.c": "39c98789f1bb56d9c643511ddd0157f3a8ff068163bbd613f10fd3def1445d7d",
    "klippy/chelper/__init__.py": "1cbd1c8059de0f9eab3f69b7306d6f820520a52437aa9d6075c8df555e7b306c",
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def atomic_write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".polar-support-", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_bytes(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".polar-support-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def git_path(klipper, name):
    path = Path(subprocess.check_output(
        ['git', '-C', str(klipper), 'rev-parse', '--git-path', name], text=True).strip())
    return path if path.is_absolute() else klipper / path


def legacy_plan(klipper):
    """Validate all old tracked files before any installation mutation."""
    legacy = klipper / ".polar-center-upgrade"
    records_file = legacy / "installed.json"
    if legacy.is_symlink():
        raise RuntimeError('Preserving unmanaged legacy state symlink')
    records = json.loads(records_file.read_text()) if records_file.is_file() else None
    plan = []
    for name in LEGACY_FILES:
        target = klipper / name
        if target.is_symlink():
            raise RuntimeError(f'Preserving unmanaged tracked-file symlink: {name}')
        staged = subprocess.run(['git', '-C', str(klipper), 'diff', '--cached',
                                 '--quiet', '--', name])
        if staged.returncode:
            raise RuntimeError(f'Preserving staged Klipper changes: {name}')
        head = subprocess.check_output(["git", "-C", str(klipper), "show", "HEAD:" + name])
        original = hashlib.sha256(head).hexdigest()
        allowed = {original, LEGACY_INSTALLED[name]}
        if records is not None:
            rec = records.get(name)
            if not rec or rec.get('original_sha256') != original:
                raise RuntimeError(f'Legacy backup no longer matches Klipper HEAD: {name}')
            backup = legacy / 'original' / name
            if digest(backup) != original:
                raise RuntimeError(f'Legacy backup checksum mismatch: {name}')
            allowed.add(rec['installed_sha256'])
        if digest(target) not in allowed:
            raise RuntimeError(f'Preserving independently modified Klipper file: {name}')
        mode = subprocess.check_output(['git', '-C', str(klipper), 'ls-tree',
                                        'HEAD', '--', name], text=True).split()[0]
        mode = 0o755 if mode == '100755' else 0o644
        if digest(target) != original or (target.stat().st_mode & 0o777) != mode:
            plan.append((name, head, mode))
    return plan


def restore_legacy(klipper, state_root):
    plan = legacy_plan(klipper)
    legacy = klipper / '.polar-center-upgrade'
    if not plan and not legacy.exists():
        return
    destination = state_root / ('legacy-' + time.strftime('%Y%m%d-%H%M%S')
                                + '-' + uuid.uuid4().hex[:8])
    destination.mkdir(parents=True)
    # Preserve patched bytes even when the old install record is missing.
    for name, _, _ in plan:
        backup = destination / 'patched' / name
        backup.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(klipper / name, backup)
    for name, data, mode in plan:
        atomic_bytes(klipper / name, data)
        (klipper / name).chmod(mode)
    if legacy.exists():
        shutil.move(str(legacy), destination / 'installer-state')
    print(f'Restored recognized legacy Polar changes; backup: {destination}')


def build(klipper):
    identity = fingerprint(klipper)
    include = klipper / "klippy/chelper"
    for name in ("itersolve.h", "trapq.h"):
        if not (include / name).is_file():
            raise RuntimeError(f"Klipper interface missing: {include / name}")
    output = ROOT / "build/polar_center.so"
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(".so.tmp")
    subprocess.run([
        "gcc", "-Wall", "-Werror", "-O2", "-shared", "-fPIC",
        "-I", str(include), "-o", str(temporary),
        str(ROOT / "csrc/polar_center.c"), "-lm",
    ], check=True)
    library = ctypes.CDLL(str(temporary))
    library.polar_center_stepper_alloc.restype = ctypes.c_void_p
    pointer = library.polar_center_stepper_alloc()
    if not pointer:
        raise RuntimeError("Polar solver allocation smoke test failed")
    ctypes.CDLL(None).free(ctypes.c_void_p(pointer))
    os.replace(temporary, output)
    atomic_write(ROOT / 'build/fingerprint', identity + '\n')


def preflight(klipper):
    fingerprint(klipper)
    legacy_plan(klipper)
    git_path(klipper, 'info/exclude')
    tracked = subprocess.check_output(
        ['git', '-C', str(klipper), 'ls-files', '--', PLUGIN_FILE], text=True)
    if tracked.strip():
        raise RuntimeError('polar_center.py is tracked in this Klipper checkout; '
                           'preserving its Git history and index')
    target = klipper / PLUGIN_FILE
    if target.is_symlink() and target.resolve() != (ROOT / PLUGIN_FILE).resolve():
        raise RuntimeError('Preserving an unmanaged polar_center.py symlink')
    allowed = {digest(ROOT / PLUGIN_FILE),
               '6e8cf8ade70dbe10275a1f031b6aae682420922881a242cf42061c959a103b7a',
               '2f82db4aa174b1f964cabaf971ada517a92db5711e1810ba25c061263faacbfd'}
    records = klipper / '.polar-center-upgrade/installed.json'
    if records.is_file():
        allowed.add(json.loads(records.read_text()).get(PLUGIN_FILE, {}).get('installed_sha256'))
    if target.is_file() and not target.is_symlink() and digest(target) not in allowed:
        raise RuntimeError('Preserving an independently modified polar_center.py')


def install_link(klipper, state_root):
    source = ROOT / PLUGIN_FILE
    target = klipper / PLUGIN_FILE
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_symlink():
        target.unlink()
    elif target.exists():
        if digest(target) != digest(source):
            backup = state_root / ("preexisting-polar-center-" + time.strftime("%Y%m%d-%H%M%S.py"))
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(target), backup)
            print(f"Preserved the previous polar_center.py at {backup}")
        else:
            target.unlink()
    target.symlink_to(source)
    exclude = git_path(klipper, 'info/exclude')
    marker = "/" + PLUGIN_FILE
    previous = exclude.read_text() if exclude.exists() else ''
    lines = previous.splitlines()
    if marker not in lines:
        atomic_write(exclude, previous.rstrip() + "\n" + marker + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--klipper", type=Path, default=Path.home() / "klipper")
    parser.add_argument("--sync", action="store_true", help="Build and refresh the managed link")
    parser.add_argument('--check', action='store_true', help='Preflight only')
    parser.add_argument('--state-root', type=Path,
                        default=Path.home() / '.local/share/klipper-polar-support')
    args = parser.parse_args()
    klipper = args.klipper.expanduser().resolve()
    if not (klipper / "klippy/klippy.py").is_file():
        raise RuntimeError(f"Not a Klipper source checkout: {klipper}")
    state_root = args.state_root
    preflight(klipper)
    if args.check:
        return
    build(klipper)
    restore_legacy(klipper, state_root)
    install_link(klipper, state_root)
    changed = subprocess.check_output(
        ["git", "-C", str(klipper), "status", "--porcelain"], text=True)
    if changed.strip():
        print("Other Klipper changes remain (preserved):\n" + changed.rstrip())
    else:
        print("Klipper Git checkout is clean; Polar support is linked and built.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
