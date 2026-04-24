#!/usr/bin/env bash
set -euo pipefail

echo "[devcontainer] ESP-IDF: $(idf.py --version 2>/dev/null || echo unavailable)"
echo "[devcontainer] esptool: $(python -m esptool version 2>/dev/null | tail -n 1 || echo unavailable)"

if compgen -G "/dev/ttyUSB*" > /dev/null || compgen -G "/dev/ttyACM*" > /dev/null; then
  echo "[devcontainer] Serial device nodes:"
  ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
else
  echo "[devcontainer] No /dev/ttyUSB* or /dev/ttyACM* devices visible in the container."
fi

if [ -d /dev/bus/usb ]; then
  echo "[devcontainer] USB bus is mounted."
else
  echo "[devcontainer] /dev/bus/usb is not mounted."
fi
