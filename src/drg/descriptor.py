"""Stage 2 - hybrid 37-D descriptor: 33-D FPFH + 4 radar channels (Sec. 3.3, Alg. 1 line 5).

    f_i = [ (1 - alpha) * fpfh_i / |fpfh_i| ,  alpha * radar_i / |radar_i| ]
    radar_i = z-scored [ RCS rho_i, confidence w_i, neighbour count n_0.3, height z_i ]

The z-score statistics (NormStats) are per sequence, computed from every 10th scan
(scripts/compute_norm_stats.py). Using the statistics of another sequence changes the mean
translation error by at most 0.14 mm (paper, Sec. 3.3).
"""
from __future__ import annotations

import copy
from dataclasses import asdict, dataclass
from typing import Iterable

import numpy as np
import open3d as o3d
import yaml
from scipy.spatial import cKDTree

from .ego_velocity import soft_reve
from .params import DRGParams

CHANNELS = ("rcs", "doppler_weight", "density", "height")


@dataclass(frozen=True)
class NormStats:
    """Mean and standard deviation of each radar channel over a sequence."""
    rcs: tuple
    doppler_weight: tuple
    density: tuple
    height: tuple

    @classmethod
    def from_yaml(cls, path: str) -> "NormStats":
        with open(path) as f:
            d = yaml.safe_load(f)["norm_stats"]
        return cls(**{c: (float(d[c]["mean"]), float(d[c]["std"])) for c in CHANNELS})

    def to_yaml(self, path: str, comment: str = "") -> None:
        body = {"norm_stats": {c: {"mean": m, "std": s} for c, (m, s) in asdict(self).items()}}
        with open(path, "w") as f:
            if comment:
                f.write("".join(f"# {line}\n" for line in comment.splitlines()))
            yaml.safe_dump(body, f, sort_keys=False)


def neighbour_count(xyz: np.ndarray, radius: float) -> np.ndarray:
    """Number of other returns within `radius` of each return (n_0.3)."""
    neighbours = cKDTree(xyz).query_ball_point(xyz, radius)
    return np.array([len(n) - 1 for n in neighbours], dtype=np.float64)


def compute_norm_stats(scans: Iterable[np.ndarray], p: DRGParams, seed: int = 42) -> NormStats:
    """Channel statistics over a set of raw scans, each an (N, 5) array x, y, z, RCS, Doppler."""
    rcs, weight, density, height = [], [], [], []
    for points in scans:
        kept, _, w = soft_reve(points, p, seed)
        if len(kept) < 5:
            continue
        rcs.append(kept[:, 3])
        weight.append(w)
        height.append(kept[:, 2])
        density.append(neighbour_count(kept[:, :3], p.density_radius))

    def mean_std(chunks):
        x = np.concatenate(chunks)
        return float(np.mean(x)), float(np.std(x))

    return NormStats(rcs=mean_std(rcs), doppler_weight=mean_std(weight),
                     density=mean_std(density), height=mean_std(height))


def fpfh(pcd: o3d.geometry.PointCloud, p: DRGParams) -> np.ndarray:
    """33-D FPFH with normals oriented towards the sensor at the origin. Returns (N, 33)."""
    pcd = copy.deepcopy(pcd)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=p.fpfh_normal_radius, max_nn=p.fpfh_normal_max_nn))
    xyz, normals = np.asarray(pcd.points), np.asarray(pcd.normals)
    flip = np.einsum("ij,ij->i", normals, -xyz) < 0
    normals[flip] *= -1.0
    pcd.normals = o3d.utility.Vector3dVector(normals)
    feature = o3d.pipelines.registration.compute_fpfh_feature(pcd, o3d.geometry.KDTreeSearchParamHybrid(
        radius=p.fpfh_feature_radius, max_nn=p.fpfh_feature_max_nn))
    return np.asarray(feature.data).T


def radar_channels(points: np.ndarray, weights: np.ndarray, stats: NormStats,
                   p: DRGParams) -> np.ndarray:
    """The four z-scored radar channels [rho, w, n_0.3, z]. Returns (N, 4)."""
    def zscore(x, mean_std):
        mean, std = mean_std
        return np.clip((x - mean) / max(std, 1e-8), -p.zscore_clip, p.zscore_clip)

    return np.column_stack([
        zscore(points[:, 3], stats.rcs),
        zscore(weights, stats.doppler_weight),
        zscore(neighbour_count(points[:, :3], p.density_radius), stats.density),
        zscore(points[:, 2], stats.height),
    ])


def hybrid_descriptor(points: np.ndarray, pcd: o3d.geometry.PointCloud, weights: np.ndarray,
                      stats: NormStats, p: DRGParams) -> np.ndarray:
    """37-D descriptor: the two blocks are L2-normalised separately, then weighted by alpha."""
    geometric = fpfh(pcd, p)
    radar = radar_channels(points, weights, stats, p)
    geometric = geometric / (np.linalg.norm(geometric, axis=1, keepdims=True) + 1e-8)
    radar = radar / (np.linalg.norm(radar, axis=1, keepdims=True) + 1e-8)
    alpha = p.radar_block_weight
    return np.hstack([(1.0 - alpha) * geometric, alpha * radar])
