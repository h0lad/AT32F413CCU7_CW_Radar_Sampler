#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-or-later
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
openocd_bin="${workspace_dir}/drivers/openocd/build-at32/src/openocd"
openocd_tcl_dir="${workspace_dir}/drivers/openocd/tcl"
project_openocd_dir="${workspace_dir}/openocd"

if [ ! -x "${openocd_bin}" ]; then
  printf 'Local OpenOCD binary not found: %s\n' "${openocd_bin}" >&2
  printf 'Build it first with ./tools/build-openocd.sh\n' >&2
  exit 1
fi

exec "${openocd_bin}" \
  -s "${openocd_tcl_dir}" \
  -s "${project_openocd_dir}" \
  -f interface/stlink.cfg \
  -f at32f413ccu7.cfg \
  -c "$*"
