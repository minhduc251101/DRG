"""Stage 6 - Doppler translation fusion (Sec. 3.6, Eqs. 7-8).

The ego-velocities of the two scans integrate (trapezoidal rule) to a translation
    t_dop = 0.5 * (v_T + R v_S) * dt ,
which is averaged with the geometric translation by inverse variance:
    t = w t_dop + (1 - w) t_geo ,   w = (sigma_v dt)^-2 / ((sigma_v dt)^-2 + sigma_g^-2).
The weight depends only on dt: at 12 Hz (dt = 0.083 s) w = 0.93, at 1 s keyframes w = 0.08.
The rotation is not changed.
"""
from __future__ import annotations

import numpy as np

from .params import DRGParams


def doppler_translation(v_source: np.ndarray, v_target: np.ndarray, R: np.ndarray,
                        dt: float) -> np.ndarray:
    """Translation of the sensor between the two scans, expressed in the target frame (Eq. 7)."""
    return 0.5 * (v_target + R @ v_source) * dt


def fusion_weight(dt: float, p: DRGParams) -> float:
    """Inverse-variance weight of the Doppler translation (Eq. 8)."""
    inv_var_doppler = 1.0 / (p.sigma_v * dt) ** 2
    return inv_var_doppler / (inv_var_doppler + 1.0 / p.sigma_g**2)


def fuse(t_geo: np.ndarray, t_dop: np.ndarray, weight: float) -> np.ndarray:
    return weight * t_dop + (1.0 - weight) * t_geo
