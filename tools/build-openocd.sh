#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-or-later
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
openocd_src_dir="${workspace_dir}/drivers/openocd"
openocd_build_dir="${openocd_src_dir}/build-at32"

for tool in autoreconf automake aclocal libtoolize pkg-config make gcc; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    printf 'Missing build tool: %s\n' "${tool}" >&2
    exit 1
  fi
done

if ! pkg-config --exists libusb-1.0; then
  printf 'Missing development package: libusb-1.0\n' >&2
  exit 1
fi

mkdir -p "${openocd_build_dir}"

if [ ! -f "${openocd_src_dir}/configure" ]; then
  (cd "${openocd_src_dir}" && ./bootstrap)
fi

(cd "${openocd_build_dir}" && \
  env CCACHE_DISABLE=1 CC=cc CXX=c++ \
  "${openocd_src_dir}/configure" \
    --srcdir="${openocd_src_dir}" \
    --disable-werror \
    --disable-doxygen-html \
    --disable-doxygen-pdf \
    --enable-stlink \
    --disable-jlink \
    --disable-ftdi \
    --disable-ti-icdi \
    --disable-ulink \
    --disable-usb-blaster-2 \
    --disable-ft232r \
    --disable-vsllink \
    --disable-xds110 \
    --disable-cmsis-dap-v2 \
    --disable-osbdm \
    --disable-opendous \
    --disable-armjtagew \
    --disable-rlink \
    --disable-usbprog \
    --disable-esp-usb-jtag \
    --disable-cmsis-dap \
    --disable-nulink \
    --disable-kitprog \
    --disable-usb-blaster \
    --disable-presto \
    --disable-openjtag \
    --disable-linuxgpiod \
    --disable-buspirate \
    --disable-parport \
    --disable-parport-ppdev \
    --disable-parport-giveio \
    --disable-jtag_vpi \
    --disable-vdebug \
    --disable-jtag_dpi \
    --disable-amtjtagaccel \
    --disable-bcm2835gpio \
    --disable-imx_gpio \
    --disable-am335xgpio \
    --disable-ep93xx \
    --disable-at91rm9200 \
    --disable-gw16012 \
    --disable-sysfsgpio \
    --disable-xlnx_pcie_xvc \
    --disable-remote-bitbang)

if [ -f "${openocd_build_dir}/jimtcl/Makefile" ]; then
  sed -i 's/^CC = ccache cc$/CC = cc/' "${openocd_build_dir}/jimtcl/Makefile"
  sed -i 's/^CXX = ccache c++$/CXX = c++/' "${openocd_build_dir}/jimtcl/Makefile"
fi

(cd "${openocd_build_dir}" && make -j"$(nproc)")

printf 'Built OpenOCD: %s\n' "${openocd_build_dir}/src/openocd"
