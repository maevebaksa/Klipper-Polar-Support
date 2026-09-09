#!/usr/bin/env python3
"""Ask Moonraker to update Polar support only while the printer is cold and idle."""
import json
import sys
import re
from urllib.parse import urlencode
from urllib.request import Request, urlopen

BASE = "http://127.0.0.1:7125"


def call(path, data=None):
    request = Request(BASE + path, headers={"Content-Type": "application/json"},
                      data=None if data is None else json.dumps(data).encode())
    with urlopen(request, timeout=900 if data else 15) as response:
        value = json.load(response)
    if "error" in value:
        raise RuntimeError("Moonraker rejected the Polar support update")
    return value["result"]


def idle():
    if call("/server/info").get("klippy_state") != "ready":
        return False
    status = call("/printer/objects/query?print_stats&idle_timeout&heaters")["status"]
    if status.get("print_stats", {}).get("state") not in ("standby", "complete", "cancelled"):
        return False
    if status.get("idle_timeout", {}).get("state") != "Idle":
        return False
    heaters = status.get("heaters", {}).get("available_heaters", [])
    if not heaters:
        return False
    targets = call("/printer/objects/query?" + urlencode({h: "target" for h in heaters}))["status"]
    if any(targets.get(h, {}).get("target") != 0 for h in heaters):
        return False
    return True


def main():
    if '--check-idle' in sys.argv:
        if not idle():
            raise SystemExit('Printer must be ready, idle, and have all heater targets off.')
        return
    if not idle():
        return
    update = call("/machine/update/status")
    info = update.get("version_info", {}).get("polar-support", {})
    if (update.get("busy") or info.get("is_valid") is not True or info.get("is_dirty")
            or info.get("corrupt") or info.get("warnings") or info.get("anomalies")):
        return
    origin = str(info.get('remote_url', '')).removesuffix('.git').lower()
    expected = 'https://github.com/maevebaksa/klipper-polar-support'
    remote, current = info.get('remote_hash', ''), info.get('current_hash', '')
    if origin != expected or not all(re.fullmatch('[0-9a-f]{40}', h) for h in (remote, current)):
        return
    if remote != current and idle():
        call("/machine/update/client", {"name": "polar-support"})


if __name__ == "__main__":
    main()
