# SPDX-License-Identifier: AGPL-3.0-or-later
from __future__ import annotations

import json
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")


FRAME_VERSION = 0x02
FRAME_FORMAT_IQ_U16_LE = 0x01
FRAME_FORMAT_META = 0x02
FRAME_FORMAT_TELEMETRY = 0x03
FRAME_HEADER_STRUCT = struct.Struct("<BBHIII")
FRAME_CRC_STRUCT = struct.Struct("<I")
META_STRUCT = struct.Struct("<4sIIIIHBBII")
DEVICE_TELEMETRY_STRUCT = struct.Struct("<IIIIIIIII4BII")
CRC32C_POLYNOMIAL_REV = 0x82F63B78
ADC_CENTER = 2048.0
SPECTROGRAM_FLOOR_DB = 0.0
SPECTROGRAM_TOP_DB = 30.0
FFT_DISPLAY_TOP_DB = 18.0
WATERFALL_DISPLAY_TOP_DB = 24.0
WATERFALL_COMPRESSION_TAU_DB = 10.0
FFT_Y_AXIS_LIMIT_TOP_DB = 72.0
FFT_DB_DISPLAY_TOP_DB = 50.0
FFT_DBM_DISPLAY_MIN_DBM = 0.0
FFT_DBM_DISPLAY_MAX_DBM = 100.0
FFT_DBM_REFERENCE_OFFSET_DB = 60.0
SPEED_OF_LIGHT_MPS = 299_792_458.0
RADAR_CARRIER_HZ = 24_250_000_000.0
DOPPLER_HZ_TO_MPS = SPEED_OF_LIGHT_MPS / (2.0 * RADAR_CARRIER_HZ)
DOPPLER_HZ_TO_KMH = DOPPLER_HZ_TO_MPS * 3.6


def build_crc32c_table() -> List[int]:
    table: List[int] = []
    for value in range(256):
        crc = value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ CRC32C_POLYNOMIAL_REV
            else:
                crc >>= 1
        table.append(crc & 0xFFFFFFFF)
    return table


CRC32C_TABLE = build_crc32c_table()
CRC_CORRECTION_MAX_BODY_BYTES = 96


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc = CRC32C_TABLE[(crc ^ byte) & 0xFF] ^ (crc >> 8)
    return (~crc) & 0xFFFFFFFF


def cobs_decode(encoded: bytes) -> bytes:
    if not encoded:
        raise ValueError("empty COBS frame")

    decoded = bytearray()
    index = 0
    encoded_length = len(encoded)

    while index < encoded_length:
        code = encoded[index]
        if code == 0:
            raise ValueError("unexpected zero inside COBS payload")

        index += 1
        next_index = index + code - 1
        if next_index > encoded_length:
            raise ValueError("truncated COBS block")

        decoded.extend(encoded[index:next_index])
        index = next_index

        if code != 0xFF and index < encoded_length:
            decoded.append(0)

    return bytes(decoded)


def parse_size(value: str) -> int:
    units = [
        ("gb", 1024 ** 3),
        ("g", 1024 ** 3),
        ("mb", 1024 ** 2),
        ("m", 1024 ** 2),
        ("kb", 1024),
        ("k", 1024),
        ("b", 1),
    ]

    text = value.strip().lower()
    for suffix, factor in units:
        if text.endswith(suffix):
            number = text[: -len(suffix)].strip()
            return int(float(number) * factor)
    return int(text)


def parse_duration(value: str) -> float:
    units = {
        "ms": 1e-3,
        "s": 1.0,
        "m": 60.0,
        "h": 3600.0,
    }

    text = value.strip().lower()
    for suffix, factor in units.items():
        if text.endswith(suffix):
            number = text[: -len(suffix)].strip()
            return float(number) * factor
    return float(text)


def format_duration(seconds: float) -> str:
    total_seconds = max(0, int(seconds))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def timestamp_for_filename() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


def bit_flip_copy(data: bytes, bit_positions: Sequence[int]) -> bytes:
    mutated = bytearray(data)
    for bit_position in bit_positions:
        byte_index = bit_position // 8
        bit_mask = 1 << (bit_position % 8)
        mutated[byte_index] ^= bit_mask
    return bytes(mutated)


def frame_to_centered_iq(frame: "RadarFrame", np_module):
    iq = np_module.frombuffer(frame.iq_payload, dtype="<u2").reshape(-1, 2)
    i = iq[:, 0].astype(np_module.float32) - ADC_CENTER
    q = iq[:, 1].astype(np_module.float32) - ADC_CENTER
    return i, q


def frames_to_centered_iq(frames: Sequence["RadarFrame"], np_module):
    if not frames:
        empty = np_module.empty(0, dtype=np_module.float32)
        return empty, empty, 0

    if len(frames) == 1:
        i, q = frame_to_centered_iq(frames[0], np_module)
        return i, q, frames[0].sample_rate_hz

    iq_chunks = [
        np_module.frombuffer(frame.iq_payload, dtype="<u2").reshape(-1, 2)
        for frame in frames
    ]
    iq = np_module.concatenate(iq_chunks, axis=0)
    i = iq[:, 0].astype(np_module.float32) - ADC_CENTER
    q = iq[:, 1].astype(np_module.float32) - ADC_CENTER
    return i, q, frames[-1].sample_rate_hz


def fft_excess_db(np_module, fft_values):
    fft_db = 20.0 * np_module.log10(np_module.maximum(np_module.abs(fft_values), 1e-6))
    baseline_db = float(np_module.median(fft_db))
    return np_module.maximum(0.0, fft_db - baseline_db)


def doppler_hz_to_mps(doppler_hz: float, direction_sign: float = 1.0) -> float:
    return float(doppler_hz) * DOPPLER_HZ_TO_MPS * float(direction_sign)


def direction_from_velocity(velocity_mps: float) -> str:
    if velocity_mps > 0.05:
        return "approaching"
    if velocity_mps < -0.05:
        return "receding"
    return "stationary"


def parse_metadata_payload(payload: bytes, telemetry: "Telemetry") -> None:
    if len(payload) < META_STRUCT.size:
        telemetry.decode_errors += 1
        return
    (
        magic,
        firmware_version,
        carrier_khz,
        uart_baud,
        sample_rate_hz,
        _frame_samples,
        adc_bits,
        adc_channels,
        _header_size,
        _payload_size,
    ) = META_STRUCT.unpack_from(payload, 0)
    if magic != b"RADR":
        telemetry.decode_errors += 1
        return
    telemetry.metadata_frames += 1
    telemetry.firmware_version = firmware_version
    telemetry.carrier_khz = carrier_khz
    telemetry.uart_baud = uart_baud
    telemetry.last_sample_rate_hz = sample_rate_hz
    telemetry.adc_bits = adc_bits
    telemetry.adc_channels = adc_channels


def parse_device_telemetry_payload(payload: bytes, telemetry: "Telemetry") -> None:
    if len(payload) < DEVICE_TELEMETRY_STRUCT.size:
        telemetry.decode_errors += 1
        return
    values = DEVICE_TELEMETRY_STRUCT.unpack_from(payload, 0)
    telemetry.device_telemetry_frames += 1
    telemetry.device_iq_frames_built = values[0]
    telemetry.device_uart_frames_completed = values[1]
    telemetry.device_adc_event_overflows = values[2]
    telemetry.device_tx_backpressure = values[3]
    telemetry.device_control_backpressure = values[4]
    telemetry.device_adc_dma_errors = values[5]
    telemetry.device_uart_dma_errors = values[6]
    telemetry.device_stream_fault = values[8]
    telemetry.device_queue_high_watermark = values[9]


@dataclass
class RadarFrame:
    version: int
    frame_format: int
    sample_pairs: int
    sequence: int
    sample_index: int
    sample_rate_hz: int
    iq_payload: bytes
    corrected_bits: int = 0


@dataclass
class Telemetry:
    frames_ok: int = 0
    frames_corrected: int = 0
    crc_errors: int = 0
    uncorrectable_errors: int = 0
    decode_errors: int = 0
    sequence_gaps: int = 0
    bytes_received: int = 0
    samples_received: int = 0
    last_sequence: Optional[int] = None
    last_sample_rate_hz: int = 0
    current_recording_name: str = "-"
    metadata_frames: int = 0
    device_telemetry_frames: int = 0
    firmware_version: int = 0
    carrier_khz: int = 24_250_000
    uart_baud: int = 0
    adc_bits: int = 12
    adc_channels: int = 2
    device_iq_frames_built: int = 0
    device_uart_frames_completed: int = 0
    device_adc_event_overflows: int = 0
    device_tx_backpressure: int = 0
    device_control_backpressure: int = 0
    device_adc_dma_errors: int = 0
    device_uart_dma_errors: int = 0
    device_stream_fault: int = 0
    device_queue_high_watermark: int = 0


class CrcCorrector:
    def __init__(self) -> None:
        self._cache: Dict[int, Tuple[List[int], Dict[int, int]]] = {}

    def _prepare(self, body_length: int) -> Tuple[List[int], Dict[int, int]]:
        cached = self._cache.get(body_length)
        if cached is not None:
            return cached

        zero_body = bytes(body_length)
        zero_crc = crc32c(zero_body)
        bit_count = body_length * 8
        syndromes: List[int] = [0] * bit_count
        lookup: Dict[int, int] = {}

        for bit_index in range(bit_count):
            error_mask = bit_flip_copy(zero_body, (bit_index,))
            syndrome = crc32c(error_mask) ^ zero_crc
            syndromes[bit_index] = syndrome
            lookup[syndrome] = bit_index

        cached = (syndromes, lookup)
        self._cache[body_length] = cached
        return cached

    def correct(self, body: bytes, expected_crc: int) -> Tuple[Optional[bytes], int]:
        computed_crc = crc32c(body)
        if computed_crc == expected_crc:
            return body, 0

        if len(body) > CRC_CORRECTION_MAX_BODY_BYTES:
            return None, -1

        delta = computed_crc ^ expected_crc
        syndromes, lookup = self._prepare(len(body))

        single_bit = lookup.get(delta)
        if single_bit is not None:
            return bit_flip_copy(body, (single_bit,)), 1

        for bit_index, syndrome in enumerate(syndromes):
            second_bit = lookup.get(delta ^ syndrome)
            if second_bit is None or second_bit <= bit_index:
                continue
            return bit_flip_copy(body, (bit_index, second_bit)), 2

        return None, -1


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> Iterable[bytes]:
        self._buffer.extend(data)
        frames: List[bytes] = []

        while True:
            try:
                delimiter_index = self._buffer.index(0)
            except ValueError:
                break

            encoded = bytes(self._buffer[:delimiter_index])
            del self._buffer[: delimiter_index + 1]
            if encoded:
                frames.append(encoded)

        return frames


class RadarRecorder:
    def __init__(
        self,
        directory: Optional[Path],
        prefix: str,
        rotate_size: Optional[int],
        rotate_duration: Optional[float],
        keep_files: Optional[int],
    ) -> None:
        self.enabled = directory is not None
        self.directory = directory
        self.prefix = prefix
        self.rotate_size = rotate_size
        self.rotate_duration = rotate_duration
        self.keep_files = keep_files
        self._file = None
        self._meta_path: Optional[Path] = None
        self._data_path: Optional[Path] = None
        self._file_started_at = 0.0
        self._file_bytes_written = 0
        self._file_index = 0
        self._latest_header: Optional[RadarFrame] = None

        if self.enabled:
            assert self.directory is not None
            self.directory.mkdir(parents=True, exist_ok=True)

    @property
    def current_name(self) -> str:
        if self._data_path is None:
            return "-"
        return self._data_path.name

    def _write_metadata(self, frame: RadarFrame, telemetry: Telemetry) -> None:
        if self._meta_path is None:
            return

        metadata = {
            "format": "iq_u16_le_interleaved",
            "frame_version": frame.version,
            "frame_format": frame.frame_format,
            "sample_pairs_per_frame": frame.sample_pairs,
            "sample_rate_hz": frame.sample_rate_hz,
            "adc_center_code": ADC_CENTER,
            "file_bytes_written": self._file_bytes_written,
            "frames_ok": telemetry.frames_ok,
            "frames_corrected": telemetry.frames_corrected,
            "crc_errors": telemetry.crc_errors,
            "uncorrectable_errors": telemetry.uncorrectable_errors,
            "sequence_gaps": telemetry.sequence_gaps,
            "generated_at_unix": time.time(),
            "data_file": self.current_name,
        }
        self._meta_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    def _prune_old_files(self) -> None:
        if not self.enabled or self.keep_files is None or self.keep_files < 1:
            return
        assert self.directory is not None

        data_files = sorted(self.directory.glob(f"{self.prefix}_*.iq16le"))
        if len(data_files) <= self.keep_files:
            return

        for stale in data_files[: len(data_files) - self.keep_files]:
            meta = stale.with_suffix(".json")
            try:
                stale.unlink()
            except FileNotFoundError:
                pass
            try:
                meta.unlink()
            except FileNotFoundError:
                pass

    def _open_new_file(self, frame: RadarFrame) -> None:
        if not self.enabled:
            return

        assert self.directory is not None
        base_name = f"{self.prefix}_{timestamp_for_filename()}_{self._file_index:04d}"
        self._file_index += 1
        self._data_path = self.directory / f"{base_name}.iq16le"
        self._meta_path = self.directory / f"{base_name}.json"
        self._file = self._data_path.open("wb")
        self._file_started_at = time.monotonic()
        self._file_bytes_written = 0
        self._latest_header = frame
        self._prune_old_files()

    def _needs_rotation(self, incoming_bytes: int) -> bool:
        if self._file is None:
            return False
        if self.rotate_size is not None and self._file_bytes_written + incoming_bytes > self.rotate_size:
            return True
        if self.rotate_duration is not None:
            if (time.monotonic() - self._file_started_at) >= self.rotate_duration:
                return True
        return False

    def write(self, frame: RadarFrame, telemetry: Telemetry) -> None:
        if not self.enabled:
            return

        if self._file is None:
            self._open_new_file(frame)
        elif self._needs_rotation(len(frame.iq_payload)):
            self.close(telemetry)
            self._open_new_file(frame)

        assert self._file is not None
        self._file.write(frame.iq_payload)
        self._file_bytes_written += len(frame.iq_payload)
        self._latest_header = frame

    def close(self, telemetry: Telemetry) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
        if self._latest_header is not None:
            self._write_metadata(self._latest_header, telemetry)


class TelemetryPrinter:
    def __init__(self) -> None:
        self._last_refresh = time.monotonic()
        self._last_frames = 0
        self._last_line_length = 0

    def print(self, telemetry: Telemetry, started_at: float, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_refresh) < 0.5:
            return

        elapsed = max(1e-6, now - self._last_refresh)
        frame_total = telemetry.frames_ok + telemetry.frames_corrected
        delta_frames = frame_total - self._last_frames
        fps = delta_frames / elapsed
        self._last_frames = frame_total
        self._last_refresh = now

        line = (
            f"FPS {fps:7.1f} | "
            f"CRC {telemetry.crc_errors:6d} | "
            f"corr {telemetry.frames_corrected:6d} | "
            f"bad {telemetry.uncorrectable_errors:6d} | "
            f"seq-gap {telemetry.sequence_gaps:6d} | "
            f"dev-drop {telemetry.device_tx_backpressure:5d}/{telemetry.device_adc_event_overflows:4d} | "
            f"dur {format_duration(now - started_at)} | "
            f"rec {telemetry.current_recording_name}"
        )

        padded = line.ljust(max(self._last_line_length, len(line)))
        sys.stdout.write("\r" + padded)
        sys.stdout.flush()
        self._last_line_length = len(padded)

    def finish(self) -> None:
        sys.stdout.write("\n")
        sys.stdout.flush()


def decode_frame(encoded_frame: bytes, corrector: CrcCorrector, telemetry: Telemetry) -> Optional[RadarFrame]:
    try:
        decoded = cobs_decode(encoded_frame)
    except ValueError:
        telemetry.decode_errors += 1
        return None

    if len(decoded) < FRAME_HEADER_STRUCT.size + FRAME_CRC_STRUCT.size:
        telemetry.decode_errors += 1
        return None

    body = decoded[:-FRAME_CRC_STRUCT.size]
    (expected_crc,) = FRAME_CRC_STRUCT.unpack(decoded[-FRAME_CRC_STRUCT.size :])
    corrected_body, corrected_bits = corrector.correct(body, expected_crc)

    if corrected_body is None:
        telemetry.crc_errors += 1
        telemetry.uncorrectable_errors += 1
        return None

    if corrected_bits > 0:
        telemetry.crc_errors += 1
        telemetry.frames_corrected += 1

    version, frame_format, sample_pairs, sequence, sample_index, sample_rate_hz = FRAME_HEADER_STRUCT.unpack_from(
        corrected_body, 0
    )

    if version != FRAME_VERSION:
        telemetry.decode_errors += 1
        return None

    payload = corrected_body[FRAME_HEADER_STRUCT.size :]
    if frame_format == FRAME_FORMAT_IQ_U16_LE:
        expected_payload_size = sample_pairs * 4
    elif frame_format in (FRAME_FORMAT_META, FRAME_FORMAT_TELEMETRY):
        expected_payload_size = sample_pairs
    else:
        telemetry.decode_errors += 1
        return None

    if len(payload) != expected_payload_size:
        telemetry.decode_errors += 1
        return None

    if frame_format == FRAME_FORMAT_META:
        parse_metadata_payload(payload, telemetry)
    elif frame_format == FRAME_FORMAT_TELEMETRY:
        parse_device_telemetry_payload(payload, telemetry)

    return RadarFrame(
        version=version,
        frame_format=frame_format,
        sample_pairs=sample_pairs,
        sequence=sequence,
        sample_index=sample_index,
        sample_rate_hz=sample_rate_hz,
        iq_payload=payload,
        corrected_bits=max(0, corrected_bits),
    )


def update_telemetry_for_frame(frame: RadarFrame, telemetry: Telemetry) -> None:
    if frame.frame_format != FRAME_FORMAT_IQ_U16_LE:
        return

    if telemetry.last_sequence is not None:
        expected_sequence = (telemetry.last_sequence + 1) & 0xFFFFFFFF
        if frame.sequence != expected_sequence:
            telemetry.sequence_gaps += (frame.sequence - expected_sequence) & 0xFFFFFFFF

    telemetry.last_sequence = frame.sequence
    telemetry.samples_received += frame.sample_pairs
    telemetry.last_sample_rate_hz = frame.sample_rate_hz

    if frame.corrected_bits == 0:
        telemetry.frames_ok += 1


def list_serial_ports() -> int:
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial fehlt. Installiere z.B. `python3 -m pip install pyserial`.", file=sys.stderr)
        return 1

    ports = list(list_ports.comports())
    if not ports:
        print("Keine seriellen Ports gefunden.")
        return 0

    for port in ports:
        description = port.description or "-"
        print(f"{port.device}\t{description}")
    return 0
