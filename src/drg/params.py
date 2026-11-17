"""All constants of DRG in one place.

The defaults are the values used for every result in the paper (Algorithm 1). They were fixed on
five tuning sets (carpark sets 1-3, garden sets 1-2) and then never changed; see docs/PARAMETERS.md
for how each one was chosen.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any, Dict

import yaml


@dataclass(frozen=True)
class DRGParams:
    # --- Stage 1: Soft-REVE ego-velocity (Sec. 3.2, Alg. 1 lines 2-4) -------------------------
    min_range: float = 0.25            # m    returns used by DRG must lie in [min_range, max_range]
    max_range: float = 100.0           # m
    min_rcs: float = 0.0               # dB   returns with RCS <= min_rcs are dropped
    max_azimuth_deg: float = 75.0      # deg  field-of-view limits of the radar
    max_elevation_deg: float = 45.0    # deg
    reve_outlier_prob: float = 0.4     # REVE defaults (Doer & Trommer): sets the RANSAC iteration count
    reve_success_prob: float = 0.9999
    reve_inlier_threshold: float = 0.15  # m/s  |Doppler residual| below which a return is a RANSAC inlier
    sigma_floor: float = 0.10          # m/s  MAD scale of the residuals is clipped to [floor, cap]
    sigma_cap: float = 0.15            # m/s
    zero_velocity: float = 0.05        # m/s  below this speed all confidence weights are 1

    # --- Stage 2: hybrid 37-D descriptor (Sec. 3.3, Alg. 1 line 5) ----------------------------
    fpfh_normal_radius: float = 0.2    # m
    fpfh_normal_max_nn: int = 30
    fpfh_feature_radius: float = 0.5   # m
    fpfh_feature_max_nn: int = 100
    density_radius: float = 0.3        # m    radius of the neighbour count n_0.3
    zscore_clip: float = 5.0           # radar channels are z-scored and clipped to +-zscore_clip
    radar_block_weight: float = 0.5    # alpha: weight of the radar block vs. the FPFH block

    # --- Stage 4: Doppler-RCS gate (Sec. 3.4, Eq. 4) -------------------------------------------
    tau_rcs: float = 1.0               # dB   max |RCS_i - RCS_j| of a kept correspondence      [tuned]
    tau_residual: float = 0.01         # m/s  max |e_i - e_j| of a kept correspondence          [tuned]

    # --- Stage 5: RANSAC + Cauchy IRLS (Sec. 3.5, Eq. 6) ---------------------------------------
    ransac_trials: int = 40            # independent seeded RANSAC runs; the best one is kept
    ransac_max_iterations: int = 50000
    ransac_confidence: float = 0.999
    ransac_edge_ratio: float = 0.9     # Open3D edge-length checker
    inlier_distance: float = 0.3       # m    RANSAC inlier distance and IRLS truncation r_max  [tuned]
    cauchy_c: float = 0.1              # m    Cauchy scale c                                     [tuned]
    irls_iterations: int = 10
    max_translation: float = 5.0       # m    hypotheses with a larger |t| are rejected
    max_roll_pitch_deg: float = 10.0   # deg  hypotheses with a larger roll or pitch are rejected
    seed: int = 42

    # --- Stage 6: Doppler translation fusion (Sec. 3.6, Eq. 8) ---------------------------------
    sigma_v: float = 0.10              # m/s  ego-velocity accuracy (nominal, from the literature)
    sigma_g: float = 0.03              # m    geometric translation accuracy (nominal, tuning sets)

    @classmethod
    def from_dict(cls, values: Dict[str, Any]) -> "DRGParams":
        known = {f.name for f in fields(cls)}
        unknown = set(values) - known
        if unknown:
            raise ValueError(f"unknown DRG parameter(s): {sorted(unknown)}")
        return cls(**values)

    @classmethod
    def from_yaml(cls, path: str) -> "DRGParams":
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f).get("drg", {}))
