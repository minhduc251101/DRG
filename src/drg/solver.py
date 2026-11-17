"""Stage 5 - geometric estimate: correspondence RANSAC + truncated-Cauchy IRLS (Sec. 3.5, Eq. 6).

For each of `ransac_trials` seeded trials:
  1. Open3D correspondence RANSAC on the gated correspondences gives T0.
  2. T0 is refined by IRLS over ALL gated correspondences with the weights
         omega_ij = w_ij / (1 + (r_ij / c)^2),  omega_ij = 0 if r_ij > r_max,
     each step solved in closed form by weighted SVD (Arun / Umeyama).
The trial with the most inliers (ties: lowest weighted RMSE) wins. Implausible hypotheses
(|t| > 5 m, roll or pitch > 10 deg) are rejected; this is the only failure check.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from .params import DRGParams

reg = o3d.pipelines.registration


@dataclass
class GeometricEstimate:
    T: np.ndarray          # 4x4, maps source points into the target frame
    n_inliers: int         # correspondences with residual < inlier_distance
    rmse: float            # weighted RMSE over those inliers


def is_plausible(T: np.ndarray, p: DRGParams) -> bool:
    """Reject |t| > max_translation or |roll|, |pitch| > max_roll_pitch_deg."""
    if np.linalg.norm(T[:3, 3]) > p.max_translation:
        return False
    roll, pitch, _ = Rotation.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
    return abs(roll) <= p.max_roll_pitch_deg and abs(pitch) <= p.max_roll_pitch_deg


def weighted_svd(A: np.ndarray, B: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Weighted least-squares rigid transform B ~ R A + t (A, B: 3xN) with reflection correction."""
    w = w / np.sum(w)
    a_mean = (A * w).sum(axis=1, keepdims=True)
    b_mean = (B * w).sum(axis=1, keepdims=True)
    H = (A - a_mean) @ np.diag(w) @ (B - b_mean).T
    U, _, Vt = np.linalg.svd(H)
    V = Vt.T
    R = V @ np.diag([1, 1, np.linalg.det(V) * np.linalg.det(U.T)]) @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3:4] = b_mean - R @ a_mean
    return T


def residuals(T: np.ndarray, A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """|R a + t - b| for every correspondence (A, B: Nx3)."""
    return np.linalg.norm((T[:3, :3] @ A.T).T + T[:3, 3] - B, axis=1)


def cauchy_irls(A: np.ndarray, B: np.ndarray, w: np.ndarray, T0: np.ndarray,
                p: DRGParams) -> np.ndarray:
    """Truncated-Cauchy IRLS from T0 over all correspondences (A, B: 3xN)."""
    T = T0.copy()
    for _ in range(p.irls_iterations):
        r = np.linalg.norm((T[:3, :3] @ A + T[:3, 3:4]) - B, axis=0)
        omega = w / (1.0 + (r / p.cauchy_c) ** 2)
        omega[r > p.inlier_distance] = 0.0
        if omega.sum() < 1e-9 or (omega > 0).sum() < 3:
            break
        T_new = weighted_svd(A, B, omega)
        if not is_plausible(T_new, p):
            break
        converged = np.abs(T_new - T).max() < 1e-7
        T = T_new
        if converged:
            break
    return T


def score(T: np.ndarray, A: np.ndarray, B: np.ndarray, w: np.ndarray, p: DRGParams):
    """(number of inliers, weighted RMSE over the inliers)."""
    r = residuals(T, A, B)
    inlier = r < p.inlier_distance
    n = int(inlier.sum())
    if n == 0:
        return 0, np.inf
    return n, float(np.sqrt(np.sum(w[inlier] * r[inlier] ** 2) / (np.sum(w[inlier]) + 1e-12)))


def ransac_cauchy(source_xyz: np.ndarray, target_xyz: np.ndarray, i: np.ndarray, j: np.ndarray,
                  w: np.ndarray, p: DRGParams) -> Optional[GeometricEstimate]:
    """Best of `ransac_trials` RANSAC + IRLS runs, or None if every hypothesis was rejected."""
    if len(i) < 3:
        return None
    source_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_xyz))
    target_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target_xyz))
    correspondences = o3d.utility.Vector2iVector(np.c_[i, j].astype(np.int32))
    A, B = source_xyz[i], target_xyz[j]

    o3d.utility.random.seed(p.seed)
    best, best_key = None, None
    for _ in range(p.ransac_trials):
        result = reg.registration_ransac_based_on_correspondence(
            source_pcd, target_pcd, correspondences, p.inlier_distance,
            reg.TransformationEstimationPointToPoint(False), 3,
            [reg.CorrespondenceCheckerBasedOnEdgeLength(p.ransac_edge_ratio),
             reg.CorrespondenceCheckerBasedOnDistance(p.inlier_distance)],
            reg.RANSACConvergenceCriteria(p.ransac_max_iterations, p.ransac_confidence))
        n_ransac_inliers = len(result.correspondence_set)
        if result.fitness <= 0 or n_ransac_inliers < 3:
            continue
        T0 = np.array(result.transformation)          # writable copy (scipy rejects read-only views)
        if not is_plausible(T0, p):
            continue

        T = cauchy_irls(A.T, B.T, w, T0, p)
        n_inliers, rmse = score(T, A, B, w, p)
        # Most inliers wins; ties go to the lower weighted RMSE, then to the better RANSAC result.
        key = (n_inliers, -rmse, float(result.fitness), n_ransac_inliers)
        if best is None or key > best_key:
            best, best_key = GeometricEstimate(T, n_inliers, rmse), key
    return best
