#!/usr/bin/env python3
"""Apply the plugin with Klipper stopped; called by the managed systemd unit."""
import argparse
from pathlib import Path
import subprocess
from auto_update import idle

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--user', required=True)
    parser.add_argument('--klipper', required=True)
    args = parser.parse_args()
    command = ['/usr/sbin/runuser', '-u', args.user, '--', '/usr/bin/python3',
               str(ROOT / 'install.py'), '--klipper', args.klipper]
    subprocess.run(command + ['--check'], check=True)
    if not idle():
        raise SystemExit('Polar update deferred: printer is not cold and idle.')
    subprocess.run(['systemctl', 'stop', 'klipper.service'], check=True)
    # On installation failure leave Klipper stopped, preserving the diagnostic
    # and preventing a partial plugin installation from initializing motion.
    subprocess.run(command + ['--sync'], check=True)
    subprocess.run(['systemctl', 'start', 'klipper.service'], check=True)


if __name__ == '__main__':
    main()
