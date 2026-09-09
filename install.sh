#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -eq 0 ]]; then
    echo "Run as the normal printer user, not root." >&2
    exit 1
fi
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
printer_user="$(id -un)"
printer_home="$(getent passwd "${printer_user}" | cut -d: -f6)"
klipper_root="${KLIPPER_DIR:-${printer_home}/klipper}"
config_root="${printer_home}/printer_data/config"
moonraker_config="${config_root}/moonraker.conf"
allowed_services="${printer_home}/printer_data/moonraker.asvc"

python3 "${project_root}/auto_update.py" --check-idle
python3 "${project_root}/install.py" --klipper "${klipper_root}" --check
if [[ ! -f ${allowed_services} || ! -f ${moonraker_config} ]]; then
    echo "Missing Moonraker configuration or allowed service file." >&2
    exit 1
fi

service_name="klipper"
if ! systemctl cat klipper.service >/dev/null 2>&1; then
    echo "The expected klipper.service was not found." >&2
    exit 1
fi

service_tmp="$(mktemp)"
timer_service_tmp="$(mktemp)"
timer_tmp="$(mktemp)"
trap 'rm -f "${service_tmp}" "${timer_service_tmp}" "${timer_tmp}"' EXIT
sed -e "s|@@USER@@|${printer_user}|g" \
    -e "s|@@PROJECT_ROOT@@|${project_root}|g" \
    -e "s|@@KLIPPER_ROOT@@|${klipper_root}|g" \
    -e "s|@@KLIPPER_SERVICE@@|${service_name}|g" \
    "${project_root}/systemd/polar-support.service.in" > "${service_tmp}"
sed -e "s|@@USER@@|${printer_user}|g" \
    -e "s|@@PROJECT_ROOT@@|${project_root}|g" \
    "${project_root}/systemd/polar-support-update.service.in" > "${timer_service_tmp}"
cp "${project_root}/systemd/polar-support-update.timer.in" "${timer_tmp}"

sudo install -m 644 "${service_tmp}" /etc/systemd/system/polar-support.service
sudo install -m 644 "${timer_service_tmp}" /etc/systemd/system/polar-support-update.service
sudo install -m 644 "${timer_tmp}" /etc/systemd/system/polar-support-update.timer

if [[ ! -f ${allowed_services} ]]; then
    echo "Missing ${allowed_services}; start/update Moonraker first." >&2
    exit 1
fi
if ! grep -qx 'polar-support' "${allowed_services}"; then
    cp -a "${allowed_services}" "${allowed_services}.before-polar-support-$(date +%Y%m%d-%H%M%S)"
    printf '%s\n' 'polar-support' >> "${allowed_services}"
fi

updater="${config_root}/polar-support-updater.conf"
python3 - "${updater}" "${project_root}" <<'PY'
from pathlib import Path
import sys
path, root = Path(sys.argv[1]), Path(sys.argv[2])
path.write_text(f'''# Managed by Klipper-Polar-Support
[update_manager polar-support]
type: git_repo
channel: dev
path: {root}
origin: https://github.com/maevebaksa/Klipper-Polar-Support.git
primary_branch: main
managed_services: polar-support
refresh_interval: 6
''')
PY
include='[include polar-support-updater.conf]'
if ! grep -Fqx "${include}" "${moonraker_config}"; then
    cp -a "${moonraker_config}" "${moonraker_config}.before-polar-support-$(date +%Y%m%d-%H%M%S)"
    printf '\n%s\n' "${include}" >> "${moonraker_config}"
fi

sudo systemctl daemon-reload
sudo systemctl start polar-support.service
sudo systemctl restart moonraker.service
sudo systemctl enable --now polar-support-update.timer
echo "Installed Polar support. Moonraker now shows polar-support in Update Manager."
