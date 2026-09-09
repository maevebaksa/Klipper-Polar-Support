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


def restore_legacy(klipper, state_root):
    legacy = klipper / ".polar-center-upgrade"
    records_file = legacy / "installed.json"
    if not records_file.is_file():
        # Recover an exact known 0.1.0 patch even if its state directory was
        # lost. HEAD is read, never reset; unrelated modifications are kept.
        head = {name: subprocess.check_output(
            ["git", "-C", str(klipper), "show", "HEAD:" + name])
                for name in LEGACY_FILES}
        current = {name: digest(klipper / name) for name in LEGACY_FILES}
        recognized = [name for name in LEGACY_FILES
                      if current[name] == LEGACY_INSTALLED[name]]
        if recognized:
            for name in LEGACY_FILES:
                original = hashlib.sha256(head[name]).hexdigest()
                if current[name] not in (LEGACY_INSTALLED[name], original):
                    raise RuntimeError(f"Preserving independently modified Klipper file: {name}")
            for name in recognized:
                atomic_bytes(klipper / name, head[name])
            print("Restored the exact legacy Polar patch from Klipper HEAD.")
        return
    records = json.loads(records_file.read_text())
    for name in LEGACY_FILES:
        rec = records.get(name)
        if not rec:
            raise RuntimeError(f"Legacy installation record is missing {name}")
        target, backup = klipper / name, legacy / "original" / name
        if digest(target) not in (rec["installed_sha256"], rec["original_sha256"]):
            raise RuntimeError(f"Preserving independently modified Klipper file: {name}")
        if digest(backup) != rec["original_sha256"]:
            raise RuntimeError(f"Legacy backup checksum mismatch: {name}")
        head = subprocess.check_output(["git", "-C", str(klipper), "show", "HEAD:" + name])
        if hashlib.sha256(head).hexdigest() != rec["original_sha256"]:
            raise RuntimeError(f"Legacy backup no longer matches Klipper HEAD: {name}")
    for name in LEGACY_FILES:
        rec = records[name]
        target = klipper / name
        if digest(target) == rec["installed_sha256"]:
            shutil.copy2(legacy / "original" / name, target)
    destination = state_root / ("legacy-patched-source-" + time.strftime("%Y%m%d-%H%M%S"))
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(legacy), destination)
    print(f"Restored tracked Klipper files; retained legacy backup at {destination}")


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
    if not (klipper / '.git/info/exclude').is_file():
        raise RuntimeError('Klipper must have .git/info/exclude')
    target = klipper / PLUGIN_FILE
    if target.is_symlink() and target.resolve() != (ROOT / PLUGIN_FILE).resolve():
        raise RuntimeError('Preserving an unmanaged polar_center.py symlink')
    allowed = {digest(ROOT / PLUGIN_FILE),
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
    exclude = klipper / ".git/info/exclude"
    if not exclude.is_file():
        raise RuntimeError("Klipper must be a Git checkout with .git/info/exclude")
    marker = "/" + PLUGIN_FILE
    lines = exclude.read_text().splitlines()
    if marker not in lines:
        atomic_write(exclude, exclude.read_text().rstrip() + "\n" + marker + "\n")


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
        ["git", "-C", str(klipper), "status", "--porcelain", "--untracked-files=no"], text=True)
    if changed.strip():
        print("WARNING: unrelated tracked Klipper modifications remain:\n" + changed.rstrip())
    else:
        print("Klipper tracked source is clean; Polar support is linked and built.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
