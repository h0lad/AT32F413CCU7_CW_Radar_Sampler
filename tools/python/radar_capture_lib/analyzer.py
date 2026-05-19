# SPDX-License-Identifier: AGPL-3.0-or-later
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Protocol, runtime_checkable


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
    targets: List[DetectedTarget]
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


@runtime_checkable
class SpeedEstimator(Protocol):

    def analyze(
        self,
        i_buffer: Deque[float],
        q_buffer: Deque[float],
        sample_rate_hz: int,
        display_window: int,
    ) -> Optional[AnalysisSnapshot]: ...

    def reset(self) -> None: ...
