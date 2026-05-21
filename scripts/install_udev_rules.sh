#!/usr/bin/env bash

set -e

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SRC="${PACKAGE_DIR}/udev/tty-usb.rules"
RULE_DST="/etc/udev/rules.d/tty-usb.rules"

echo "[INFO] imu_pkg directory: ${PACKAGE_DIR}"

if [ ! -f "${RULE_SRC}" ]; then
    echo "[ERROR] udev rule file not found: ${RULE_SRC}"
    exit 1
fi

echo "[INFO] Installing EBIMU udev rule..."
echo "[INFO] ${RULE_SRC} -> ${RULE_DST}"

sudo cp "${RULE_SRC}" "${RULE_DST}"
sudo chmod 644 "${RULE_DST}"

echo "[INFO] Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[INFO] Restarting udev service..."
sudo systemctl restart udev

echo "[INFO] Done."
echo ""
echo "Check symbolic link:"
echo "  ls -l /dev/ttyUSB-EBIMU"
echo ""
echo "If the device is already connected, unplug and reconnect it."
