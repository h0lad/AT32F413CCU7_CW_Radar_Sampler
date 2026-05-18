# SPDX-License-Identifier: AGPL-3.0-or-later
from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Sequence

from .core import (
    CrcCorrector,
    FRAME_FORMAT_IQ_U16_LE,
    FrameParser,
    RadarFrame,
    RadarRecorder,
    Telemetry,
    TelemetryPrinter,
    decode_frame,
    list_serial_ports,
    parse_duration,
    parse_size,
    update_telemetry_for_frame,
)
from .ui import LivePlotter, PyQtGraphRadarStudio

VISUAL_BATCH_MAX_FRAMES = 64
MAX_DRAIN_CHUNKS_PER_LOOP = 32
MAX_DRAIN_MS_PER_LOOP = 8.0


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Radar I/Q capture and analysis for the AT32 UART stream.",
    )
    parser.add_argument("port", nargs="?", help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=6_000_000, help="Serial baud rate")
    parser.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    parser.add_argument("--stop-after", type=parse_duration, help="Stop after a duration like 30s, 10m, 1h")
    parser.add_argument("--no-visual", action="store_true", help="Disable live plots and run headless")
    parser.add_argument("--ui", action="store_true", help="Launch the PyQtGraph user interface")
    parser.add_argument("--window-samples", type=int, default=32768, help="Samples kept for the live plots")
    parser.add_argument("--fft-size", type=int, default=16384, help="FFT size for spectrum and spectrogram")
    parser.add_argument("--refresh-hz", type=float, default=8.0, help="Maximum plot refresh rate")
    parser.add_argument("--record-dir", type=Path, help="Enable recording into this directory")
    parser.add_argument("--record-prefix", default="radar_iq", help="Recording file prefix")
    parser.add_argument("--rotate-size", type=parse_size, help="Rotate recording after a file reaches this size")
    parser.add_argument(
        "--rotate-duration",
        type=parse_duration,
        help="Rotate recording after this duration per file",
    )
    parser.add_argument("--keep-files", type=int, help="Keep only the newest N recording files")
    return parser.parse_args(argv)


def create_visualizer(args: argparse.Namespace):
    if args.ui and args.no_visual:
        raise ValueError("--ui und --no-visual schliessen sich gegenseitig aus.")
    if args.ui:
        return PyQtGraphRadarStudio(
            window_samples=args.window_samples,
            fft_size=args.fft_size,
            refresh_hz=args.refresh_hz,
            source_name=args.port or "serial",
            recording_enabled=args.record_dir is not None,
        )
    if args.no_visual:
        return None
    return LivePlotter(
        window_samples=args.window_samples,
        fft_size=args.fft_size,
        refresh_hz=args.refresh_hz,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    if args.list_ports:
        return list_serial_ports()

    if not args.port:
        print("A serial port is required unless --list-ports is used.", file=sys.stderr)
        return 2

    try:
        import serial
    except ImportError:
        print("pyserial fehlt. Installiere z.B. `python3 -m pip install pyserial`.", file=sys.stderr)
        return 1

    try:
        visualizer = create_visualizer(args)
    except Exception as exc:
        print(f"Visualisierung konnte nicht gestartet werden: {exc}", file=sys.stderr)
        return 1

    recorder = RadarRecorder(
        directory=args.record_dir,
        prefix=args.record_prefix,
        rotate_size=args.rotate_size,
        rotate_duration=args.rotate_duration,
        keep_files=args.keep_files,
    )

    parser = FrameParser()
    corrector = CrcCorrector()
    telemetry = Telemetry(current_recording_name=recorder.current_name)
    printer = TelemetryPrinter()
    started_at = time.monotonic()
    pending_visual_frames: Deque[RadarFrame] = deque(maxlen=256)
    serial_port = None
    serial_error: Optional[BaseException] = None

    try:
        serial_port = serial.Serial(args.port, baudrate=args.baud, timeout=0)
        serial_port.reset_input_buffer()

        def process_chunk(chunk: bytes) -> None:
            if chunk:
                telemetry.bytes_received += len(chunk)

            for encoded_frame in parser.feed(chunk):
                frame = decode_frame(encoded_frame, corrector, telemetry)
                if frame is None:
                    continue

                update_telemetry_for_frame(frame, telemetry)
                if frame.frame_format != FRAME_FORMAT_IQ_U16_LE:
                    continue

                recorder.write(frame, telemetry)
                telemetry.current_recording_name = recorder.current_name

                if visualizer is not None:
                    pending_visual_frames.append(frame)

        def update_visualizer(now: float) -> None:
            if visualizer is None:
                return

            should_draw = True
            if hasattr(visualizer, "last_draw") and hasattr(visualizer, "refresh_period"):
                should_draw = (now - visualizer.last_draw) >= visualizer.refresh_period

            if not should_draw and len(pending_visual_frames) < VISUAL_BATCH_MAX_FRAMES:
                return

            if pending_visual_frames:
                frames = list(pending_visual_frames)[-VISUAL_BATCH_MAX_FRAMES:]
                pending_visual_frames.clear()
                if hasattr(visualizer, "update_batch"):
                    visualizer.update_batch(frames)
                else:
                    for frame in frames:
                        visualizer.update(frame)

            if should_draw:
                visualizer.update_telemetry(telemetry, started_at)
                visualizer.draw()
                if hasattr(visualizer, "process_events"):
                    visualizer.process_events()

        while True:
            if visualizer is not None and hasattr(visualizer, "process_events"):
                visualizer.process_events()

            if args.stop_after is not None and (time.monotonic() - started_at) >= args.stop_after:
                break

            if visualizer is not None and hasattr(visualizer, "is_active") and not visualizer.is_active():
                break

            drain_started_at = time.monotonic()
            drained_chunks = 0
            while drained_chunks < MAX_DRAIN_CHUNKS_PER_LOOP:
                chunk = serial_port.read(65536)
                if not chunk:
                    break
                process_chunk(chunk)
                drained_chunks += 1
                if ((time.monotonic() - drain_started_at) * 1000.0) >= MAX_DRAIN_MS_PER_LOOP:
                    break

            now = time.monotonic()
            update_visualizer(now)

            printer.print(telemetry, started_at)
    except KeyboardInterrupt:
        pass
    except (serial.SerialException, OSError) as exc:
        serial_error = exc
    finally:
        if serial_port is not None:
            serial_port.close()
        recorder.close(telemetry)
        telemetry.current_recording_name = recorder.current_name
        if visualizer is not None:
            visualizer.close()
        printer.print(telemetry, started_at, force=True)
        printer.finish()

    if serial_error is not None:
        print(f"Serial stream stopped: {serial_error}", file=sys.stderr)
        return 3

    return 0
