# SPDX-License-Identifier: AGPL-3.0-or-later
#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

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


@dataclass
class AnalysisSnapshot:
    sample_axis: object
    i: object
    q: object
    magnitude: object
    phase: object
    dominant_hz: float
    peak_snr_db: float
    detector_state: str
    velocity_mps: float
    velocity_kmh: float
    direction_label: str
    rms_value: float
    phase_std: float
    motion_index: float
    spectral_spread: float
    coherence: float
    scene_label: str
    scene_confidence: float
    background_noise_avg_dbm: float
    background_noise_max_dbm: float
    targets: List["DetectedTarget"]
    freq_axis: Optional[object] = None
    fft_magnitude: Optional[object] = None
    fft_dbm: Optional[object] = None
    cfar_threshold: Optional[object] = None
    cfar_noise_floor: Optional[object] = None
    cfar_threshold_dbm: Optional[object] = None
    cfar_noise_floor_dbm: Optional[object] = None
    background_profile_db: Optional[object] = None
    background_profile_dbm: Optional[object] = None
    spur_profile_db: Optional[object] = None
    filter_attenuation_db: Optional[object] = None
    notch_mask: Optional[object] = None
    doppler_min: float = 0.0
    doppler_max: float = 0.0
    tracker_predicted_hz: float = 0.0
    tracker_gate_hz: float = 0.0
    tracker_innovation_hz: float = 0.0
    tracker_score: float = 0.0
    tracker_phase: str = "search"


@dataclass
class DetectedTarget:
    bin_index: int
    doppler_hz: float
    display_db: float
    absolute_dbm: float
    snr_db: float
    velocity_mps: float
    velocity_kmh: float
    direction_label: str
    lock_score: float = 0.0
    support_width_hz: float = 0.0


class RadarAnalyzer:
    def __init__(self, np_module, fft_size: int) -> None:
        self._np = np_module
        self._fft_size = fft_size
        self._remove_dc = True
        self._doppler_span_hz: Optional[float] = 2500.0
        self._direction_sign = 1.0
        self._notch_width_hz = 50.0
        self._multi_target_enabled = False
        self._max_targets = 3
        self._detector_locked = False
        self._detector_missing = 0
        self._pending_doppler_hz: Optional[float] = None
        self._pending_hits = 0
        self._tracked_doppler_hz = 0.0
        self._tracked_snr_db = 0.0
        self._detector_acquire_snr_db = 16.0
        self._detector_release_snr_db = 11.0
        self._detector_min_prominence_db = 3.5
        self._detector_min_display_db = 6.0
        self._peak_min_separation_bins = 6
        self._display_floor_margin_db = 3.0
        self._background_profile_db = None
        self._static_line_profile_db = None
        self._last_analyze_time: Optional[float] = None
        self._startup_calibration_started_at: Optional[float] = None
        self._static_line_warmup_s = 120.0
        self._startup_calibration_s = 10.0
        self._startup_calibration_done = False
        self._startup_avg_noise_samples = []
        self._startup_max_noise_samples = []
        self._startup_static_line_samples = []
        self._autotune_generation = 0
        self._background_time_constant_s = 60.0
        self._static_line_time_constant_s = 12.0
        self._background_update_ceiling_db = 4.0
        self._threshold_window_bins = 41
        self._threshold_margin_db = 4.0
        self._static_line_window_bins = 17
        self._static_line_margin_db = 1.2
        self._static_line_floor_db = 3.5
        self._static_line_reject_margin_db = 7.5
        self._static_line_keep_db = 1.0
        self._static_line_max_attenuation_db = 24.0
        self._noise_guard_mode = "adaptive"
        self._noise_guard_avg_margin_db = 7.0
        self._noise_guard_max_margin_db = 2.5
        self._track_state = None
        self._track_cov = None
        self._last_track_time: Optional[float] = None
        self._track_process_noise_hz = 140.0
        self._track_measurement_noise_hz = 95.0
        self._track_gate_hz = 220.0
        self._track_confirm_hits = 3
        self._tracker_predicted_hz = 0.0
        self._tracker_gate_hz = 0.0
        self._tracker_innovation_hz = 0.0
        self._track_score = 0.0
        self._track_status = "search"
        self._track_score_rise = 1.35
        self._track_score_decay = 0.30
        self._track_score_hit_snr_weight = 0.22
        self._track_score_hit_prom_weight = 0.18
        self._track_score_hit_gate_weight = 0.45
        self._track_score_confirm = 5.5
        self._track_score_lock = 8.0
        self._track_score_keep = 2.0
        self._track_score_drop = 0.35
        self._track_coast_updates = 12
        self._track_probe_gate_scale = 0.65
        self._track_probe_snr_relax_db = 7.0
        self._track_probe_prom_relax_db = 2.5
        self._track_probe_min_snr_db = 4.0
        self._track_probe_min_prom_db = 1.0
        self._track_probe_energy_min_db = 2.0
        self._track_probe_sigma_scale = 0.30

    @property
    def fft_size(self) -> int:
        return self._fft_size

    @property
    def remove_dc(self) -> bool:
        return self._remove_dc

    @remove_dc.setter
    def remove_dc(self, value: bool) -> None:
        self._remove_dc = bool(value)
        self.reset_detector()

    @property
    def doppler_span_hz(self) -> Optional[float]:
        return self._doppler_span_hz

    @doppler_span_hz.setter
    def doppler_span_hz(self, value: Optional[float]) -> None:
        self._doppler_span_hz = value

    @property
    def direction_sign(self) -> float:
        return self._direction_sign

    @direction_sign.setter
    def direction_sign(self, value: float) -> None:
        self._direction_sign = float(value)

    @property
    def notch_width_hz(self) -> float:
        return self._notch_width_hz

    @notch_width_hz.setter
    def notch_width_hz(self, value: float) -> None:
        self._notch_width_hz = float(value)
        self._reset_tracking_state()

    def set_fft_size(self, value: int) -> None:
        self._fft_size = max(256, int(value))
        self.reset_detector()

    @property
    def detector_acquire_snr_db(self) -> float:
        return self._detector_acquire_snr_db

    @detector_acquire_snr_db.setter
    def detector_acquire_snr_db(self, value: float) -> None:
        self._detector_acquire_snr_db = float(value)

    @property
    def detector_release_snr_db(self) -> float:
        return self._detector_release_snr_db

    @detector_release_snr_db.setter
    def detector_release_snr_db(self, value: float) -> None:
        self._detector_release_snr_db = float(value)

    @property
    def noise_guard_mode(self) -> str:
        return self._noise_guard_mode

    @noise_guard_mode.setter
    def noise_guard_mode(self, value: str) -> None:
        self._noise_guard_mode = value if value in {"adaptive", "conservative"} else "adaptive"

    @property
    def noise_guard_avg_margin_db(self) -> float:
        return self._noise_guard_avg_margin_db

    @noise_guard_avg_margin_db.setter
    def noise_guard_avg_margin_db(self, value: float) -> None:
        self._noise_guard_avg_margin_db = max(0.0, float(value))

    @property
    def noise_guard_max_margin_db(self) -> float:
        return self._noise_guard_max_margin_db

    @noise_guard_max_margin_db.setter
    def noise_guard_max_margin_db(self, value: float) -> None:
        self._noise_guard_max_margin_db = max(0.0, float(value))

    @property
    def autotune_generation(self) -> int:
        return self._autotune_generation

    @property
    def startup_calibration_progress(self) -> float:
        if self._startup_calibration_started_at is None:
            return 0.0
        if self._startup_calibration_done:
            return 1.0
        elapsed = max(0.0, time.monotonic() - self._startup_calibration_started_at - self._static_line_warmup_s)
        return min(1.0, elapsed / self._startup_calibration_s)

    @property
    def multi_target_enabled(self) -> bool:
        return self._multi_target_enabled

    @multi_target_enabled.setter
    def multi_target_enabled(self, value: bool) -> None:
        self._multi_target_enabled = bool(value)

    @property
    def max_targets(self) -> int:
        return self._max_targets

    @max_targets.setter
    def max_targets(self, value: int) -> None:
        self._max_targets = max(1, min(8, int(value)))

    def reset_detector(self) -> None:
        self._reset_tracking_state()
        self._background_profile_db = None
        self._static_line_profile_db = None
        self._last_analyze_time = None
        self._startup_calibration_started_at = None
        self._startup_calibration_done = False
        self._startup_avg_noise_samples = []
        self._startup_max_noise_samples = []
        self._startup_static_line_samples = []

    def _reset_tracking_state(self) -> None:
        self._detector_locked = False
        self._detector_missing = 0
        self._pending_doppler_hz = None
        self._pending_hits = 0
        self._tracked_doppler_hz = 0.0
        self._tracked_snr_db = 0.0
        self._track_state = None
        self._track_cov = None
        self._last_track_time = None
        self._tracker_predicted_hz = 0.0
        self._tracker_gate_hz = 0.0
        self._tracker_innovation_hz = 0.0
        self._track_score = 0.0
        self._track_status = "search"

    def _doppler_limits(self, freq_axis) -> Tuple[float, float]:
        full_min = float(freq_axis[0])
        full_max = float(freq_axis[-1])
        if self._doppler_span_hz is None:
            return full_min, full_max
        span = min(float(self._doppler_span_hz), max(abs(full_min), abs(full_max)))
        return -span, span

    def _slow_profile_alpha(self, now: float, time_constant_s: float) -> float:
        if self._last_analyze_time is None:
            return 1.0
        dt = max(1e-3, now - self._last_analyze_time)
        return min(1.0, 1.0 - self._np.exp(-dt / max(1e-3, time_constant_s)))

    def _moving_average(self, values, window_bins: int):
        np = self._np
        window_bins = max(3, int(window_bins) | 1)
        kernel = np.ones(window_bins, dtype=np.float32) / float(window_bins)
        return np.convolve(values, kernel, mode="same")

    def _narrow_peak_mask(self, values):
        np = self._np
        values = np.asarray(values, dtype=np.float32)
        if values.size < 3:
            return values > 0.0
        mask = np.zeros(values.shape, dtype=bool)
        center = values[1:-1]
        left = values[:-2]
        right = values[2:]
        mask[1:-1] = np.logical_and(center > left, center >= right)
        return mask

    def _condition_spectrum(self, fft_values, freq_axis):
        np = self._np
        now = time.monotonic()
        if self._startup_calibration_started_at is None:
            self._startup_calibration_started_at = now
        fft_db = 20.0 * np.log10(np.maximum(np.abs(fft_values), 1e-6))
        notch_mask = np.abs(freq_axis) <= self._notch_width_hz
        background_alpha = self._slow_profile_alpha(now, self._background_time_constant_s)
        static_line_alpha = self._slow_profile_alpha(now, self._static_line_time_constant_s)

        if self._background_profile_db is None or self._background_profile_db.shape != fft_db.shape:
            self._background_profile_db = np.asarray(fft_db, dtype=np.float32).copy()
        else:
            update_source = np.minimum(fft_db, self._background_profile_db + self._background_update_ceiling_db)
            self._background_profile_db = (
                (1.0 - background_alpha) * self._background_profile_db + background_alpha * update_source
            ).astype(np.float32, copy=False)

        baseline_db = float(np.median(self._background_profile_db[~notch_mask])) if np.any(~notch_mask) else float(np.median(self._background_profile_db))
        excess_db = np.maximum(0.0, fft_db - self._background_profile_db - self._display_floor_margin_db)
        line_floor = self._moving_average(excess_db, self._static_line_window_bins)
        line_strength = np.maximum(0.0, excess_db - line_floor - self._static_line_margin_db)
        line_strength = np.where(self._narrow_peak_mask(line_strength), line_strength, 0.0)
        if self._static_line_profile_db is None or self._static_line_profile_db.shape != excess_db.shape:
            self._static_line_profile_db = np.asarray(line_strength, dtype=np.float32).copy()
        else:
            self._static_line_profile_db = (
                (1.0 - static_line_alpha) * self._static_line_profile_db + static_line_alpha * line_strength
            ).astype(np.float32, copy=False)
        static_line_attenuation_db = np.clip(
            np.maximum(0.0, self._static_line_profile_db - self._static_line_keep_db),
            0.0,
            self._static_line_max_attenuation_db,
        ).astype(np.float32, copy=False)
        fft_magnitude = np.maximum(0.0, excess_db - static_line_attenuation_db)
        if self._notch_width_hz > 0.0:
            fft_magnitude[notch_mask] = SPECTROGRAM_FLOOR_DB

        self._last_analyze_time = now
        return (
            fft_magnitude.astype(np.float32, copy=False),
            notch_mask,
            fft_db,
            baseline_db,
            self._background_profile_db.copy(),
            self._static_line_profile_db.copy(),
            static_line_attenuation_db,
            None,
        )

    def _fft_db_to_display_dbm(self, fft_db):
        np = self._np
        return np.clip(
            np.asarray(fft_db, dtype=np.float32) - FFT_DBM_REFERENCE_OFFSET_DB,
            FFT_DBM_DISPLAY_MIN_DBM,
            FFT_DBM_DISPLAY_MAX_DBM,
        )

    def _release_detector(self) -> None:
        self._detector_locked = False
        self._detector_missing = 0
        self._pending_doppler_hz = None
        self._pending_hits = 0
        self._tracked_doppler_hz = 0.0
        self._tracked_snr_db = 0.0
        self._track_score = 0.0
        self._track_status = "search"
        self._track_state = None
        self._track_cov = None
        self._tracker_predicted_hz = 0.0
        self._tracker_gate_hz = 0.0
        self._tracker_innovation_hz = 0.0

    def _decay_track_score(self, amount: float) -> None:
        self._track_score = max(0.0, self._track_score - amount)
        if self._track_score < self._track_score_drop:
            self._track_status = "search"
        elif self._detector_locked:
            self._track_status = "coasting"
        elif self._track_score >= self._track_score_confirm:
            self._track_status = "tentative"
        else:
            self._track_status = "search"

    def _measurement_quality_score(self, target: DetectedTarget, innovation_hz: float, gate_hz: float) -> float:
        normalized_gate = max(0.0, 1.0 - abs(innovation_hz) / max(1e-3, gate_hz))
        support_penalty = max(0.0, target.support_width_hz - 180.0) / 220.0
        score = (
            self._track_score_rise
            + self._track_score_hit_snr_weight * max(0.0, target.snr_db - self._detector_release_snr_db)
            + self._track_score_hit_prom_weight * max(0.0, target.lock_score)
            + self._track_score_hit_gate_weight * normalized_gate
            - support_penalty
        )
        return max(0.2, score)

    def _predicted_track_measurement(
        self,
        fft_magnitude,
        fft_db,
        freq_axis,
        search_mask,
        cfar_noise_floor,
        cfar_threshold,
        sample_rate_hz: int,
        conservative_floor_db,
        static_line_profile_db,
    ) -> Optional[DetectedTarget]:
        if self._track_state is None:
            return None
        np = self._np
        predicted_hz = float(self._track_state[0])
        bin_width_hz = max(1.0, float(sample_rate_hz) / float(self._fft_size))
        gate_hz = max(40.0, self._tracker_gate_hz * self._track_probe_gate_scale)
        gate_mask = np.logical_and(search_mask, np.abs(freq_axis - predicted_hz) <= gate_hz)
        gate_indices = np.flatnonzero(gate_mask)
        if gate_indices.size == 0:
            return None

        relaxed_snr_db = max(self._track_probe_min_snr_db, self._detector_release_snr_db - self._track_probe_snr_relax_db)
        relaxed_prom_db = max(self._track_probe_min_prom_db, self._detector_min_prominence_db - self._track_probe_prom_relax_db)
        relaxed_threshold = np.asarray(cfar_threshold, dtype=np.float32) - relaxed_prom_db
        gate_freq = np.asarray(freq_axis[gate_indices], dtype=np.float32)
        gate_signal = np.asarray(fft_magnitude[gate_indices], dtype=np.float32)
        gate_floor = np.asarray(cfar_noise_floor[gate_indices], dtype=np.float32)
        gate_lock = np.asarray(relaxed_threshold[gate_indices], dtype=np.float32)
        probe_excess = np.maximum(0.0, gate_signal - np.maximum(gate_floor, gate_lock))
        sigma_hz = max(2.0 * bin_width_hz, gate_hz * self._track_probe_sigma_scale)
        weights = np.exp(-0.5 * ((gate_freq - predicted_hz) / sigma_hz) ** 2).astype(np.float32, copy=False)
        weighted_excess = probe_excess * weights
        probe_energy = float(np.sum(weighted_excess))
        if probe_energy < self._track_probe_energy_min_db:
            return None
        centroid_hz = float(np.sum(gate_freq * weighted_excess) / max(1e-6, probe_energy))
        centroid_bin = int(np.argmin(np.abs(freq_axis - centroid_hz)))
        search_left = max(0, centroid_bin - 2)
        search_right = min(int(freq_axis.size), centroid_bin + 3)
        local_indices = np.arange(search_left, search_right, dtype=np.int32)
        if local_indices.size == 0:
            return None
        best_local_index = int(local_indices[int(np.argmax(fft_magnitude[local_indices]))])
        if abs(float(freq_axis[best_local_index]) - predicted_hz) <= gate_hz:
            preferred_bin = best_local_index
        else:
            preferred_bin = centroid_bin

        best_target: Optional[DetectedTarget] = None
        best_key = None
        probe_priority = {preferred_bin}
        ordered_gate_indices = []
        for bin_index in gate_indices:
            if int(bin_index) in probe_priority:
                ordered_gate_indices.append(int(bin_index))
        for bin_index in gate_indices:
            if int(bin_index) not in probe_priority:
                ordered_gate_indices.append(int(bin_index))
        for rank, bin_index in enumerate(ordered_gate_indices):
            local_slice = fft_magnitude[max(0, bin_index - 1) : min(fft_magnitude.size, bin_index + 2)]
            is_local_peak = float(fft_magnitude[bin_index]) >= float(np.max(local_slice))
            if not is_local_peak and rank > 0:
                continue
            signal_db = float(fft_magnitude[bin_index])
            noise_floor_db = float(cfar_noise_floor[bin_index]) if np.isfinite(cfar_noise_floor[bin_index]) else 0.0
            threshold_db = float(cfar_threshold[bin_index]) if np.isfinite(cfar_threshold[bin_index]) else noise_floor_db + self._threshold_margin_db
            peak_abs_db = float(fft_db[bin_index])
            snr_db = max(0.0, signal_db - noise_floor_db)
            prominence_db = max(0.0, signal_db - threshold_db)
            static_line_db = float(static_line_profile_db[bin_index]) if static_line_profile_db is not None else 0.0
            if snr_db < relaxed_snr_db or prominence_db < relaxed_prom_db:
                continue
            if conservative_floor_db is not None and peak_abs_db < (conservative_floor_db - 1.5):
                continue
            if static_line_db >= self._static_line_floor_db and signal_db <= (static_line_db + 2.0):
                continue

            support_width_hz = self._candidate_support_width_hz(fft_magnitude, int(bin_index), threshold_db, sample_rate_hz)
            target = DetectedTarget(
                bin_index=int(bin_index),
                doppler_hz=float(freq_axis[bin_index]),
                display_db=signal_db,
                absolute_dbm=float(np.clip(peak_abs_db - FFT_DBM_REFERENCE_OFFSET_DB, FFT_DBM_DISPLAY_MIN_DBM, FFT_DBM_DISPLAY_MAX_DBM)),
                snr_db=snr_db,
                    velocity_mps=doppler_hz_to_mps(float(freq_axis[bin_index]), self._direction_sign),
                    velocity_kmh=doppler_hz_to_mps(float(freq_axis[bin_index]), self._direction_sign) * 3.6,
                    direction_label=direction_from_velocity(doppler_hz_to_mps(float(freq_axis[bin_index]), self._direction_sign)),
                    lock_score=prominence_db + min(3.0, 0.35 * probe_energy),
                    support_width_hz=support_width_hz,
                )
            innovation_hz = target.doppler_hz - predicted_hz
            key = (
                self._measurement_quality_score(target, innovation_hz, gate_hz),
                -abs(innovation_hz),
                target.snr_db,
            )
            if best_key is None or key > best_key:
                best_key = key
                best_target = target
        return best_target

    def _synthetic_track_target(self) -> DetectedTarget:
        doppler_hz = float(self._track_state[0]) if self._track_state is not None else self._tracked_doppler_hz
        velocity_mps = doppler_hz_to_mps(doppler_hz, self._direction_sign)
        velocity_kmh = velocity_mps * 3.6
        return DetectedTarget(
            bin_index=0,
            doppler_hz=doppler_hz,
            display_db=0.0,
            absolute_dbm=float(self._np.clip(self._tracked_snr_db, FFT_DBM_DISPLAY_MIN_DBM, FFT_DBM_DISPLAY_MAX_DBM)),
            snr_db=self._tracked_snr_db,
            velocity_mps=velocity_mps,
            velocity_kmh=velocity_kmh,
            direction_label=direction_from_velocity(velocity_mps),
            lock_score=self._track_score,
            support_width_hz=0.0,
        )

    def _background_noise_guard_db(self, background_profile_db, search_mask, spur_mask):
        np = self._np
        if background_profile_db is None:
            return None, None, None
        valid_mask = np.logical_and(search_mask, np.isfinite(background_profile_db))
        if spur_mask is not None:
            valid_mask = np.logical_and(valid_mask, np.logical_not(spur_mask))
        if not np.any(valid_mask):
            valid_mask = np.logical_and(search_mask, np.isfinite(background_profile_db))
        if not np.any(valid_mask):
            return None, None, None
        noise_values = np.asarray(background_profile_db[valid_mask], dtype=np.float32)
        avg_db = float(np.mean(noise_values))
        max_db = float(np.max(noise_values))
        conservative_floor_db = max(avg_db + self._noise_guard_avg_margin_db, max_db + self._noise_guard_max_margin_db)
        return avg_db, max_db, conservative_floor_db

    def _apply_startup_autotune(self) -> None:
        if self._startup_calibration_done:
            return
        self._startup_calibration_done = True
        if not self._startup_avg_noise_samples or not self._startup_max_noise_samples:
            self._autotune_generation += 1
            return

        np = self._np
        avg_noise_db = float(np.median(np.asarray(self._startup_avg_noise_samples, dtype=np.float32)))
        max_noise_db = float(np.percentile(np.asarray(self._startup_max_noise_samples, dtype=np.float32), 90.0))
        static_line_db = (
            float(np.percentile(np.asarray(self._startup_static_line_samples, dtype=np.float32), 90.0))
            if self._startup_static_line_samples
            else 0.0
        )
        noise_span_db = max(0.0, max_noise_db - avg_noise_db)

        self._detector_acquire_snr_db = float(min(22.0, max(10.0, 10.0 + 0.70 * noise_span_db + 0.25 * static_line_db)))
        self._detector_release_snr_db = float(min(self._detector_acquire_snr_db - 1.5, max(6.0, self._detector_acquire_snr_db - 4.0)))
        self._detector_min_prominence_db = float(min(6.0, max(2.5, 2.5 + 0.30 * noise_span_db + 0.15 * static_line_db)))
        self._noise_guard_avg_margin_db = float(min(12.0, max(3.5, 3.5 + 0.45 * noise_span_db + 0.15 * static_line_db)))
        self._noise_guard_max_margin_db = float(min(6.0, max(1.0, 1.0 + 0.30 * noise_span_db + 0.10 * static_line_db)))
        self._noise_guard_mode = "conservative" if (noise_span_db >= 6.0 or static_line_db >= 5.0) else "adaptive"
        self._autotune_generation += 1

    def _local_threshold(self, fft_magnitude):
        np = self._np
        local_floor = self._moving_average(fft_magnitude, self._threshold_window_bins)
        threshold = np.maximum(self._detector_min_display_db, local_floor + self._threshold_margin_db)
        return local_floor.astype(np.float32, copy=False), threshold.astype(np.float32, copy=False)

    def _candidate_support_width_hz(self, fft_magnitude, bin_index: int, threshold_db: float, sample_rate_hz: int) -> float:
        support_floor_db = max(threshold_db, float(fft_magnitude[bin_index]) - 6.0)
        left = int(bin_index)
        right = int(bin_index)
        while left > 0 and float(fft_magnitude[left - 1]) >= support_floor_db:
            left -= 1
        last_index = int(fft_magnitude.size - 1)
        while right < last_index and float(fft_magnitude[right + 1]) >= support_floor_db:
            right += 1
        bin_width_hz = max(1.0, float(sample_rate_hz) / float(self._fft_size))
        return float(right - left + 1) * bin_width_hz

    def _describe_environment(self, background_avg_db, background_max_db, static_line_profile_db) -> Tuple[str, float]:
        if background_avg_db is None or background_max_db is None:
            return "environment unknown", 0.0
        span_db = max(0.0, float(background_max_db) - float(background_avg_db))
        static_line_db = 0.0
        if static_line_profile_db is not None:
            finite_static = self._np.asarray(static_line_profile_db, dtype=self._np.float32)
            finite_static = finite_static[self._np.isfinite(finite_static)]
            if finite_static.size:
                static_line_db = float(self._np.percentile(finite_static, 90.0))
        clutter_index = max(span_db, static_line_db)
        if clutter_index < 3.0:
            return "environment stable", 0.88
        if clutter_index < 7.0:
            return "environment moderate clutter", 0.82
        return "environment heavy clutter", 0.86

    def _find_targets(
        self,
        fft_magnitude,
        fft_db,
        freq_axis,
        search_mask,
        sample_rate_hz: int,
        conservative_floor_db=None,
        static_line_profile_db=None,
    ):
        np = self._np
        valid_indices = np.flatnonzero(search_mask)
        if valid_indices.size == 0:
            return [], np.full_like(fft_magnitude, np.nan, dtype=np.float32), np.full_like(fft_magnitude, np.nan, dtype=np.float32)
        targets: List[DetectedTarget] = []
        local_floor, local_threshold = self._local_threshold(fft_magnitude)
        cfar_threshold = np.full_like(fft_magnitude, np.nan, dtype=np.float32)
        cfar_noise_floor = np.full_like(fft_magnitude, np.nan, dtype=np.float32)
        cfar_threshold[valid_indices] = local_threshold[valid_indices]
        cfar_noise_floor[valid_indices] = local_floor[valid_indices]
        active_mask = np.logical_and(search_mask, fft_magnitude >= local_threshold)
        active_indices = np.flatnonzero(active_mask)
        if active_indices.size == 0:
            return [], cfar_threshold, cfar_noise_floor

        bin_width_hz = max(1.0, float(sample_rate_hz) / float(self._fft_size))
        regions = []
        region_start = int(active_indices[0])
        region_prev = int(active_indices[0])
        for raw_index in active_indices[1:]:
            index = int(raw_index)
            if index == (region_prev + 1):
                region_prev = index
                continue
            regions.append((region_start, region_prev))
            region_start = index
            region_prev = index
        regions.append((region_start, region_prev))

        for start_index, end_index in regions:
            region = slice(start_index, end_index + 1)
            region_signal = np.asarray(fft_magnitude[region], dtype=np.float32)
            region_floor = np.asarray(local_floor[region], dtype=np.float32)
            region_threshold = np.asarray(local_threshold[region], dtype=np.float32)
            region_freq = np.asarray(freq_axis[region], dtype=np.float32)
            region_fft_db = np.asarray(fft_db[region], dtype=np.float32)
            region_peak_offset = int(np.argmax(region_signal))
            peak_index = start_index + region_peak_offset
            signal_db = float(region_signal[region_peak_offset])
            noise_floor_db = float(region_floor[region_peak_offset])
            threshold_db = float(region_threshold[region_peak_offset])
            peak_abs_db = float(region_fft_db[region_peak_offset])
            snr_db = max(0.0, signal_db - noise_floor_db)
            prominence_db = max(0.0, signal_db - threshold_db)
            static_line_db = float(static_line_profile_db[peak_index]) if static_line_profile_db is not None else 0.0
            support_width_hz = float(end_index - start_index + 1) * bin_width_hz
            region_excess = np.maximum(0.0, region_signal - np.maximum(region_floor, region_threshold))
            if float(np.sum(region_excess)) > 1e-6:
                doppler_hz = float(np.sum(region_freq * region_excess) / np.sum(region_excess))
            else:
                doppler_hz = float(freq_axis[peak_index])
            if (
                snr_db < self._detector_release_snr_db
                or prominence_db < self._detector_min_prominence_db
                or signal_db < threshold_db
            ):
                continue
            if conservative_floor_db is not None and peak_abs_db < conservative_floor_db:
                continue
            if static_line_db >= self._static_line_floor_db and signal_db <= (static_line_db + self._static_line_reject_margin_db):
                continue
            structure_bonus = min(3.0, 0.10 * support_width_hz / max(1.0, bin_width_hz))
            targets.append(
                DetectedTarget(
                    bin_index=int(peak_index),
                    doppler_hz=doppler_hz,
                    display_db=signal_db,
                    absolute_dbm=float(np.clip(peak_abs_db - FFT_DBM_REFERENCE_OFFSET_DB, FFT_DBM_DISPLAY_MIN_DBM, FFT_DBM_DISPLAY_MAX_DBM)),
                    snr_db=snr_db,
                    velocity_mps=doppler_hz_to_mps(doppler_hz, self._direction_sign),
                    velocity_kmh=doppler_hz_to_mps(doppler_hz, self._direction_sign) * 3.6,
                    direction_label=direction_from_velocity(doppler_hz_to_mps(doppler_hz, self._direction_sign)),
                    lock_score=prominence_db + structure_bonus,
                    support_width_hz=support_width_hz,
                )
            )
        targets.sort(key=lambda target: (target.snr_db, target.lock_score), reverse=True)
        selected: List[DetectedTarget] = []
        for target in targets:
            if any(abs(target.bin_index - kept.bin_index) < self._peak_min_separation_bins for kept in selected):
                continue
            selected.append(target)
            if len(selected) >= self._max_targets:
                break
        return selected, cfar_threshold, cfar_noise_floor

    def _select_primary_target(
        self,
        candidates: List[DetectedTarget],
        fft_magnitude,
        fft_db,
        freq_axis,
        search_mask,
        cfar_noise_floor,
        cfar_threshold,
        conservative_floor_db,
        static_line_profile_db,
        sample_rate_hz: int,
    ) -> Optional[DetectedTarget]:
        now = time.monotonic()
        if self._last_track_time is None:
            dt = 1.0 / 8.0
        else:
            dt = max(1e-3, now - self._last_track_time)
        self._last_track_time = now

        if self._detector_locked and self._track_state is not None and self._track_cov is not None:
            np = self._np
            transition = np.asarray([[1.0, dt], [0.0, 1.0]], dtype=np.float32)
            drive = np.asarray([[0.5 * dt * dt], [dt]], dtype=np.float32)
            process_q = (self._track_process_noise_hz ** 2) * (drive @ drive.T)
            self._track_state = transition @ self._track_state
            self._track_cov = transition @ self._track_cov @ transition.T + process_q
            self._tracker_predicted_hz = float(self._track_state[0])
            self._tracker_gate_hz = max(
                self._track_gate_hz,
                3.0 * float(np.sqrt(max(1e-6, self._track_cov[0, 0]))),
            )
        else:
            self._tracker_predicted_hz = 0.0
            self._tracker_gate_hz = self._track_gate_hz
            self._tracker_innovation_hz = 0.0

        if self._detector_locked:
            gated = [
                target
                for target in candidates
                if abs(target.doppler_hz - self._tracker_predicted_hz) <= self._tracker_gate_hz
                and target.snr_db >= max(self._detector_release_snr_db - 6.0, 5.0)
            ]
            predicted_probe = self._predicted_track_measurement(
                fft_magnitude,
                fft_db,
                freq_axis,
                search_mask,
                cfar_noise_floor,
                cfar_threshold,
                sample_rate_hz,
                conservative_floor_db,
                static_line_profile_db,
            )
            if predicted_probe is not None:
                gated.append(predicted_probe)
            if gated:
                measurement = max(gated, key=lambda target: (target.snr_db, target.lock_score))
                is_probe_measurement = predicted_probe is not None and measurement is predicted_probe
                measurement_noise_scale = 1.75 if is_probe_measurement else 1.0
                measurement_noise = (self._track_measurement_noise_hz * measurement_noise_scale) ** 2
                innovation = measurement.doppler_hz - float(self._track_state[0])
                innovation_cov = float(self._track_cov[0, 0] + measurement_noise)
                kalman_gain = self._track_cov[:, 0] / innovation_cov
                self._track_state = self._track_state + kalman_gain * innovation
                identity = self._np.eye(2, dtype=self._np.float32)
                self._track_cov = (identity - self._np.outer(kalman_gain, self._np.asarray([1.0, 0.0], dtype=self._np.float32))) @ self._track_cov
                self._tracker_innovation_hz = float(innovation)
                self._tracked_doppler_hz = float(self._track_state[0])
                self._tracked_snr_db = measurement.snr_db
                self._detector_missing = 0
                score_gain = self._measurement_quality_score(measurement, innovation, self._tracker_gate_hz)
                if is_probe_measurement:
                    score_gain *= 0.65
                self._track_score = min(
                    20.0,
                    self._track_score + score_gain,
                )
                self._track_status = "confirmed" if self._track_score >= self._track_score_lock else "tentative"
                measurement.doppler_hz = self._tracked_doppler_hz
                measurement.velocity_mps = doppler_hz_to_mps(measurement.doppler_hz, self._direction_sign)
                measurement.velocity_kmh = measurement.velocity_mps * 3.6
                measurement.direction_label = direction_from_velocity(measurement.velocity_mps)
                return measurement
            self._detector_missing += 1
            self._decay_track_score(self._track_score_decay)
            self._tracked_snr_db *= 0.97
            if self._detector_missing >= self._track_coast_updates or self._track_score < self._track_score_keep:
                self._release_detector()
                return None
            self._track_status = "coasting"
            return self._synthetic_track_target()

        if not candidates:
            self._decay_track_score(0.35)
            return None

        best = candidates[0]
        if best.snr_db < self._detector_acquire_snr_db:
            self._pending_doppler_hz = None
            self._pending_hits = 0
            self._decay_track_score(0.25)
            return None
        bin_width_hz = max(1.0, float(sample_rate_hz) / float(self._fft_size))
        confirmation_band_hz = max(self._track_gate_hz, 4.0 * bin_width_hz)
        if self._pending_doppler_hz is None or abs(best.doppler_hz - self._pending_doppler_hz) > confirmation_band_hz:
            self._pending_doppler_hz = best.doppler_hz
            self._pending_hits = 1
            self._track_score = max(self._track_score, 0.75)
            self._track_status = "tentative"
            return None
        self._pending_hits += 1
        self._pending_doppler_hz = (self._pending_doppler_hz + best.doppler_hz) * 0.5
        self._track_score = min(
            20.0,
            self._track_score + self._measurement_quality_score(best, best.doppler_hz - self._pending_doppler_hz, confirmation_band_hz),
        )
        self._track_status = "tentative"
        if self._pending_hits < self._track_confirm_hits and self._track_score < self._track_score_confirm:
            return None
        self._detector_locked = True
        self._tracked_doppler_hz = self._pending_doppler_hz
        self._tracked_snr_db = best.snr_db
        self._track_state = self._np.asarray([self._tracked_doppler_hz, 0.0], dtype=self._np.float32)
        velocity_sigma = max(20.0, self._track_gate_hz / max(dt, 1e-2))
        self._track_cov = self._np.asarray(
            [[self._track_gate_hz ** 2, 0.0], [0.0, velocity_sigma ** 2]],
            dtype=self._np.float32,
        )
        self._tracker_predicted_hz = self._tracked_doppler_hz
        self._tracker_gate_hz = self._track_gate_hz
        self._tracker_innovation_hz = 0.0
        self._track_score = max(self._track_score, self._track_score_lock)
        self._track_status = "confirmed"
        self._pending_doppler_hz = None
        self._pending_hits = 0
        self._detector_missing = 0
        best.doppler_hz = self._tracked_doppler_hz
        best.velocity_mps = doppler_hz_to_mps(best.doppler_hz, self._direction_sign)
        best.velocity_kmh = best.velocity_mps * 3.6
        best.direction_label = direction_from_velocity(best.velocity_mps)
        return best

    def analyze(self, i_buffer: Deque[float], q_buffer: Deque[float], sample_rate_hz: int, display_window: int) -> Optional[AnalysisSnapshot]:
        if len(i_buffer) < 16:
            return None

        np = self._np
        i = np.fromiter(i_buffer, dtype=np.float32)
        q = np.fromiter(q_buffer, dtype=np.float32)
        now = time.monotonic()
        magnitude = np.hypot(i, q)
        complex_signal = i + 1j * q
        phase = np.unwrap(np.angle(complex_signal))
        if self._remove_dc:
            i = i - i.mean()
            q = q - q.mean()
            complex_signal = i + 1j * q
            magnitude = np.hypot(i, q)
            phase = np.unwrap(np.angle(complex_signal))
        phase -= phase.mean()

        display_count = min(i.size, display_window)
        view_slice = slice(-display_count, None)
        sample_axis = np.arange(display_count, dtype=np.int32)

        snapshot = AnalysisSnapshot(
            sample_axis=sample_axis,
            i=i[view_slice],
            q=q[view_slice],
            magnitude=magnitude[view_slice],
            phase=phase[view_slice],
            dominant_hz=0.0,
            peak_snr_db=0.0,
            detector_state="searching",
            velocity_mps=0.0,
            velocity_kmh=0.0,
            direction_label="stationary",
            rms_value=float(np.sqrt(np.mean(magnitude[view_slice] ** 2))),
            phase_std=float(np.std(phase[view_slice])),
            motion_index=float(np.mean(np.abs(np.diff(phase[view_slice])))),
            spectral_spread=0.0,
            coherence=float(np.abs(np.mean(complex_signal[view_slice] / np.maximum(np.abs(complex_signal[view_slice]), 1e-6)))),
            scene_label="quiet background",
            scene_confidence=0.0,
            background_noise_avg_dbm=0.0,
            background_noise_max_dbm=0.0,
            targets=[],
            cfar_threshold=None,
            cfar_noise_floor=None,
            fft_dbm=None,
            cfar_threshold_dbm=None,
            cfar_noise_floor_dbm=None,
            background_profile_db=None,
            background_profile_dbm=None,
            spur_profile_db=None,
            filter_attenuation_db=None,
        )

        if complex_signal.size < self._fft_size:
            return snapshot

        fft_input = complex_signal[-self._fft_size :]
        if self._remove_dc:
            fft_input = fft_input - fft_input.mean()
        window = np.hanning(self._fft_size)
        fft_values = np.fft.fftshift(np.fft.fft(fft_input * window))
        freq_axis = np.fft.fftshift(np.fft.fftfreq(self._fft_size, d=1.0 / max(1, sample_rate_hz)))
        doppler_min, doppler_max = self._doppler_limits(freq_axis)
        (
            fft_magnitude,
            notch_mask,
            fft_db,
            baseline_db,
            background_profile_db,
            static_line_profile_db,
            static_line_attenuation_db,
            _instant_line_db,
        ) = self._condition_spectrum(fft_values, freq_axis)
        calibration_elapsed_s = (
            0.0
            if self._startup_calibration_started_at is None
            else max(0.0, now - self._startup_calibration_started_at)
        )
        warmup_active = calibration_elapsed_s < self._static_line_warmup_s
        calibration_phase_elapsed_s = max(0.0, calibration_elapsed_s - self._static_line_warmup_s)
        calibrating = calibration_phase_elapsed_s < self._startup_calibration_s

        search_mask = np.logical_and(freq_axis >= doppler_min, freq_axis <= doppler_max)
        search_mask = np.logical_and(search_mask, np.isfinite(fft_magnitude))
        if self._notch_width_hz > 0.0:
            search_mask = np.logical_and(search_mask, np.logical_not(notch_mask))
        background_avg_db, background_max_db, conservative_floor_db = self._background_noise_guard_db(
            background_profile_db,
            search_mask,
            None,
        )
        if (not warmup_active) and calibrating:
            if background_avg_db is not None:
                self._startup_avg_noise_samples.append(float(background_avg_db))
            if background_max_db is not None:
                self._startup_max_noise_samples.append(float(background_max_db))
            if static_line_profile_db is not None:
                finite_static = self._np.asarray(static_line_profile_db, dtype=self._np.float32)
                finite_static = finite_static[self._np.isfinite(finite_static)]
                if finite_static.size:
                    self._startup_static_line_samples.append(float(self._np.max(finite_static)))
        elif (not warmup_active) and (not self._startup_calibration_done):
            self._apply_startup_autotune()
        candidates, cfar_threshold, cfar_noise_floor = self._find_targets(
            fft_magnitude,
            fft_db,
            freq_axis,
            search_mask,
            sample_rate_hz,
            conservative_floor_db=conservative_floor_db if self._noise_guard_mode == "conservative" else None,
            static_line_profile_db=static_line_profile_db,
        )
        snapshot.scene_label, snapshot.scene_confidence = self._describe_environment(
            background_avg_db,
            background_max_db,
            static_line_profile_db,
        )
        primary_target = None if (warmup_active or calibrating) else self._select_primary_target(
            candidates,
            fft_magnitude,
            fft_db,
            freq_axis,
            search_mask,
            cfar_noise_floor,
            cfar_threshold,
            conservative_floor_db,
            static_line_profile_db,
            sample_rate_hz,
        )
        snapshot.targets = candidates if self._multi_target_enabled else candidates[:1]
        snapshot.cfar_threshold = cfar_threshold
        snapshot.cfar_noise_floor = cfar_noise_floor
        snapshot.fft_dbm = self._fft_db_to_display_dbm(fft_db)
        snapshot.background_profile_db = background_profile_db
        snapshot.background_profile_dbm = self._fft_db_to_display_dbm(background_profile_db)
        snapshot.spur_profile_db = static_line_profile_db
        snapshot.filter_attenuation_db = static_line_attenuation_db
        if background_avg_db is not None:
            snapshot.background_noise_avg_dbm = float(
                self._fft_db_to_display_dbm(self._np.asarray([background_avg_db], dtype=self._np.float32))[0]
            )
        if background_max_db is not None:
            snapshot.background_noise_max_dbm = float(
                self._fft_db_to_display_dbm(self._np.asarray([background_max_db], dtype=self._np.float32))[0]
            )
        if cfar_threshold is not None:
            threshold_abs_db = cfar_threshold + background_profile_db + self._display_floor_margin_db
            snapshot.cfar_threshold_dbm = self._fft_db_to_display_dbm(threshold_abs_db)
        if cfar_noise_floor is not None:
            noise_abs_db = cfar_noise_floor + background_profile_db + self._display_floor_margin_db
            snapshot.cfar_noise_floor_dbm = self._fft_db_to_display_dbm(noise_abs_db)
        if warmup_active:
            snapshot.detector_state = "warming"
        elif calibrating:
            snapshot.detector_state = "calibrating"
        elif primary_target is not None and self._track_status == "confirmed":
            snapshot.dominant_hz = primary_target.doppler_hz
            snapshot.peak_snr_db = primary_target.snr_db
            snapshot.velocity_mps = primary_target.velocity_mps
            snapshot.velocity_kmh = primary_target.velocity_kmh
            snapshot.direction_label = primary_target.direction_label
            snapshot.detector_state = "locked"
        elif self._track_status == "coasting":
            snapshot.detector_state = "coasting"
        elif self._track_status == "tentative":
            snapshot.detector_state = "tentative"

        spectral_weights = np.maximum(np.abs(fft_values), 1e-6)
        if self._notch_width_hz > 0.0:
            spectral_weights[notch_mask] = 0.0
        spectral_weight_sum = max(1e-6, float(np.sum(spectral_weights)))
        centroid = float(np.sum(np.abs(freq_axis) * spectral_weights) / spectral_weight_sum)
        snapshot.spectral_spread = float(
            np.sqrt(np.sum(((np.abs(freq_axis) - centroid) ** 2) * spectral_weights) / spectral_weight_sum)
        )
        snapshot.freq_axis = freq_axis
        snapshot.fft_magnitude = fft_magnitude
        snapshot.notch_mask = notch_mask
        snapshot.doppler_min = doppler_min
        snapshot.doppler_max = doppler_max
        snapshot.tracker_predicted_hz = self._tracker_predicted_hz
        snapshot.tracker_gate_hz = self._tracker_gate_hz
        snapshot.tracker_innovation_hz = self._tracker_innovation_hz
        snapshot.tracker_score = self._track_score
        snapshot.tracker_phase = self._track_status
        return snapshot


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
