"""Stage 1 - Soft-REVE: ego-velocity from Doppler with a soft per-return confidence (Sec. 3.2).

A static target seen from a sensor moving with velocity v has the Doppler (range rate)
    d_i = -n_i . v ,        n_i = p_i / |p_i| .
REVE (Doer & Trommer) fits v by three-point RANSAC + least squares. Soft-REVE keeps that fit but,
instead of discarding the outliers, gives every return a weight w_i = exp(-e_i^2 / 2 sigma^2) on
its Doppler residual, so no return is thrown away.
"""
from __future__ import annotations

import math

import numpy as np

from .params import DRGParams

RANSAC_SAMPLE_SIZE = 3


def field_of_view_mask(points: np.ndarray, p: DRGParams) -> np.ndarray:
    """Returns inside the range / RCS / azimuth / elevation limits (Alg. 1 line 2)."""
    x, y, z, rcs = points[:, 0], points[:, 1], points[:, 2], points[:, 3]
    r = np.sqrt(x**2 + y**2 + z**2) + 1e-8
    azimuth = np.arctan2(y, x)
    elevation = np.arcsin(np.clip(z / r, -1, 1))
    return ((r > p.min_range) & (r < p.max_range) & (rcs > p.min_rcs)
            & (np.abs(azimuth) < np.radians(p.max_azimuth_deg))
            & (np.abs(elevation) < np.radians(p.max_elevation_deg)))


def ransac_iterations(outlier_prob: float, success_prob: float, sample_size: int) -> int:
    """Number of RANSAC iterations that finds an all-inlier sample with probability success_prob."""
    return int(math.ceil(math.log(1.0 - success_prob)
                         / math.log(1.0 - (1.0 - outlier_prob) ** sample_size)))


def soft_reve(points: np.ndarray, p: DRGParams, seed: int):
    """Estimate the ego-velocity of one scan and a confidence weight for each kept return.

    Args:
        points: (N, 5) array of x, y, z [m], RCS [dB], Doppler [m/s] in the sensor frame.
    Returns:
        kept:     (K, 5) returns inside the field-of-view limits
        velocity: (3,) sensor velocity in the sensor frame [m/s]
        weights:  (K,) confidence w_i in (0, 1]
    """
    kept = points[field_of_view_mask(points, p)]
    if len(kept) < RANSAC_SAMPLE_SIZE:
        return kept, np.zeros(3), np.ones(len(kept))

    r = np.sqrt(kept[:, 0]**2 + kept[:, 1]**2 + kept[:, 2]**2) + 1e-8
    directions = kept[:, :3] / r[:, None]          # n_i
    n_dot_v = -kept[:, 4]                          # a static return has n_i . v = -d_i

    # Three-point RANSAC least squares, then a refit on all inliers (REVE).
    rng = np.random.default_rng(seed)
    velocity, best_inliers = np.zeros(3), 0
    for _ in range(ransac_iterations(p.reve_outlier_prob, p.reve_success_prob, RANSAC_SAMPLE_SIZE)):
        sample = rng.choice(len(kept), RANSAC_SAMPLE_SIZE, replace=False)
        try:
            v = np.linalg.lstsq(directions[sample], n_dot_v[sample], rcond=None)[0]
        except np.linalg.LinAlgError:
            continue
        n_inliers = int(np.sum(np.abs(n_dot_v - directions @ v) < p.reve_inlier_threshold))
        if n_inliers > best_inliers:
            velocity, best_inliers = v, n_inliers

    inliers = np.abs(n_dot_v - directions @ velocity) < p.reve_inlier_threshold
    if np.sum(inliers) >= RANSAC_SAMPLE_SIZE:
        velocity = np.linalg.lstsq(directions[inliers], n_dot_v[inliers], rcond=None)[0]

    # Soft confidence: Gaussian kernel on the residual, scale = MAD of the static majority (Eq. 2).
    residuals = np.abs(n_dot_v - directions @ velocity)
    sigma = np.clip(1.4826 * float(np.median(residuals)), p.sigma_floor, p.sigma_cap)
    if np.linalg.norm(velocity) < p.zero_velocity:
        weights = np.ones(len(kept))
    else:
        weights = np.exp(-residuals**2 / (2.0 * sigma**2))
    return kept, velocity, weights


def doppler_residual(points: np.ndarray, velocity: np.ndarray) -> np.ndarray:
    """Ego-motion-compensated Doppler e_i = d_i + n_i . v (Eq. 2); ~0 for a static return."""
    r = np.linalg.norm(points[:, :3], axis=1) + 1e-8
    return points[:, 4] + (points[:, :3] / r[:, None]) @ velocity
