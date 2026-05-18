# AT32F413CCU7 Radar Firmware

This is an alternative firmware for [cheap CW Radar boards from Aliexpress](http://de.aliexpress.com/item/1005010388578617.html) that are based on a Artery AT32F413CCU7 microcontroller.

Features:
 - 6MBaud UART
 - Synchronous 120kHz@12-Bit sampling of the I/Q signals + Meta 
 - COPS Encoding
 - 32-Bit CRC
 - Python script for recording and visualizing the signals

With my FTDI FT2232HL board I usually stick around 0 CRC errors.

On [my website](https://www.holad.de/2026/04/26/fun-in-space-an-cw-radars/) you can find a few experiments I did with this thing.

## Dependencies

```
sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi cmake make libusb-1.0-0-dev
```

Submodules:

```
git submodule update --init --recursive
```

Build local OpenOCD (once, required for flashing):

```
./tools/build-openocd.sh
```

## Build

```
cmake --preset debug
cmake --build --preset debug
```

Artifacts: `build/debug/artifacts/<module>/{.elf,.hex,.bin,.map}`

## Flash

```
./tools/run-openocd.sh program build/debug/artifacts/radar/radar.elf verify reset exit
```

## New module

```cmake
# module/<name>/CMakeLists.txt
at32_add_module(<name>
  SOURCES src/main.c
)
```

`cmake --preset debug` picks it up automatically.

## Recording

Install Python dependencies:

```
pip install -r tools/requirements.txt
```

Capture and record:

```
python3 tools/radar_capture.py /dev/ttyUSB0 --baud 6000000 \
  --record-dir recordings --rotate-size 512M --keep-files 8
```

Live UI:

```
python3 tools/radar_capture.py /dev/ttyUSB0 --baud 6000000 --ui
```

Headless (no plots):

```
python3 tools/radar_capture.py /dev/ttyUSB0 --baud 6000000 --no-visual
```

Recording files land in `recordings/` as `<prefix>_<timestamp>_<n>.iq16le` with a sidecar `.json`.

## Licenses

This code is AGPL-3.0-or-later licensed.

| | Upstream Licenses |
|---|---|
| AT32F413_Firmware_Library | BSD-3-Clause |
| OpenOCD (AT32 fork) | GPL-2.0 |
| CMSIS | Apache-2.0 |
