#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-or-later
set -euo pipefail

port="${1:-/dev/ttyACM0}"
baud="${2:-115200}"

if command -v tio >/dev/null 2>&1; then
  exec tio -b "${baud}" "${port}"
fi

if command -v picocom >/dev/null 2>&1; then
  exec picocom --baud "${baud}" "${port}"
fi

if command -v python3 >/dev/null 2>&1 && python3 -c "import serial.tools.miniterm" >/dev/null 2>&1; then
  exec python3 -m serial.tools.miniterm "${port}" "${baud}"
fi

printf 'No supported serial monitor found. Install tio, picocom, or pyserial.\n' >&2
exit 1
