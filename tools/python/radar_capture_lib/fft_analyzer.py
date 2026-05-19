# SPDX-License-Identifier: AGPL-3.0-or-later
from __future__ import annotations

import time
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

from .analyzer import AnalysisSnapshot, DetectedTarget
from .core import (
    ADC_CENTER,
    FFT_DBM_DISPLAY_MAX_DBM,
    FFT_DBM_DISPLAY_MIN_DBM,
    FFT_DBM_REFERENCE_OFFSET_DB,
    SPECTROGRAM_FLOOR_DB,
    direction_from_velocity,
    doppler_hz_to_mps,
)


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

    def reset(self) -> None:
        self.reset_detector()

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
