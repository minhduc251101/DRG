"""DRG: registration of one pair of consecutive 4D radar scans (Algorithm 1 of the paper).

    drg = DRG(DRGParams(), NormStats.from_yaml("config/norm_stats/cp.yaml"))
    target = drg.prepare(points_t)          # stages 1-2, once per scan
    source = drg.prepare(points_s)
    result = drg.register(source, target, dt=t_s - t_t)   # stages 3-6
    result.T                                # 4x4, maps source points into the target frame

Each scan is an (N, 5) array: x, y, z [m], RCS [dB], Doppler [m/s] (signed range rate, not
ego-compensated), in the sensor frame. A scan can be prepared once and reused as the source of
one pair and the target of the next; this gives exactly the same result as preparing it twice.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from .correspondence import doppler_rcs_gate, mutual_nearest_neighbours
from .descriptor import NormStats, hybrid_descriptor
from .ego_velocity import doppler_residual, soft_reve
from .fusion import doppler_translation, fuse, fusion_weight
from .params import DRGParams
from .solver import ransac_cauchy

MIN_POINTS = 5


@dataclass
class PreparedScan:
    """One scan after stages 1-2. Arrays are aligned: row k of each belongs to the same return."""
    xyz: np.ndarray          # (K, 3) positions [m]
    rcs: np.ndarray          # (K,)   RCS [dB]
    residual: np.ndarray     # (K,)   Doppler residual e_k [m/s]
    weight: np.ndarray       # (K,)   Soft-REVE confidence w_k
    velocity: np.ndarray     # (3,)   ego-velocity [m/s]
    descriptor: np.ndarray   # (K, 37)


@dataclass
class RegistrationResult:
    T: np.ndarray                   # 4x4 final estimate (identity if registration failed)
    success: bool
    t_geo: np.ndarray               # geometric translation before fusion
    t_dop: np.ndarray               # Doppler translation (Eq. 7)
    fusion_weight: float            # w of Eq. 8
    n_putative: int                 # mutual nearest neighbours
    n_gated: int                    # correspondences kept by the gate
    n_inliers: int                  # inliers of the winning hypothesis


class DRG:
    def __init__(self, params: DRGParams, norm_stats: NormStats):
        self.p = params
        self.stats = norm_stats

    def prepare(self, points: np.ndarray) -> Optional[PreparedScan]:
        """Stages 1-2: Soft-REVE ego-velocity, Doppler residuals, weights and 37-D descriptors."""
        kept, velocity, weight = soft_reve(points, self.p, self.p.seed)
        if len(kept) < MIN_POINTS:
            return None
        xyz = kept[:, :3].astype(np.float64)
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
        descriptor = hybrid_descriptor(kept, pcd, weight, self.stats, self.p)

        # The gate attributes are looked up by position, as in the implementation used for the
        # paper: returns that share exactly the same position all take the RCS, residual and
        # weight of one of them. Only scans with duplicated positions are affected (e.g. the
        # first garden keyframes); keeping it makes the results match the paper bit for bit.
        same_position = cKDTree(kept[:, :3]).query(xyz)[1]
        residual = doppler_residual(kept, velocity)
        return PreparedScan(
            xyz=xyz,
            rcs=kept[same_position, 3],
            residual=residual[same_position],
            weight=weight[same_position],
            velocity=velocity,
            descriptor=descriptor,
        )

    def register(self, source: Optional[PreparedScan], target: Optional[PreparedScan],
                 dt: float) -> RegistrationResult:
        """Stages 3-6. `dt` = t_source - t_target in seconds."""
        if source is None or target is None:
            return self._failure(0, 0)

        # Stage 3: putative correspondences = mutual nearest neighbours in R^37.
        i, j = mutual_nearest_neighbours(source.descriptor, target.descriptor)
        n_putative = len(i)
        # Stage 4: Doppler-RCS gate and correspondence weights w_ij.
        i, j, w = doppler_rcs_gate(i, j, source, target, self.p)
        # Stage 5: RANSAC + truncated-Cauchy IRLS -> R, t_geo.
        geo = ransac_cauchy(source.xyz, target.xyz, i, j, w, self.p)
        if geo is None:
            return self._failure(n_putative, len(i))

        # Stage 6: fuse t_geo with the Doppler translation; R is kept.
        T = geo.T.copy()
        t_geo = T[:3, 3].copy()
        t_dop = doppler_translation(source.velocity, target.velocity, T[:3, :3], dt)
        weight = fusion_weight(dt, self.p)
        T[:3, 3] = fuse(t_geo, t_dop, weight)
        return RegistrationResult(T, True, t_geo, t_dop, weight, n_putative, len(i), geo.n_inliers)

    @staticmethod
    def _failure(n_putative: int, n_gated: int) -> RegistrationResult:
        nan3 = np.full(3, np.nan)
        return RegistrationResult(np.eye(4), False, nan3, nan3, 0.0, n_putative, n_gated, 0)
