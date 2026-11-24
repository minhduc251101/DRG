"""Per-pair errors and trajectory files, exactly as scored in the paper (Sec. 4.1).

Ground truth (TUM: t x y z qx qy qz qw, 10 Hz) is interpolated at the radar stamps: linear in
position, SLERP in rotation. A frame is scored only if a ground-truth sample lies within
`max_gap` seconds (0.1 s; 0.15 s on nyl). With T_err = T_gt T_est^-1:
    TE = |t_err|  [m],    RE = arccos((tr R_err - 1) / 2)  [deg].
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def pose_matrix(xyz, quat_xyzw) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = xyz
    T[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    return T


def load_tum(path: str):
    """TUM file -> (stamps (N,), poses (N, 4, 4)). Comment lines start with '#'."""
    data = np.loadtxt(path, comments="#", ndmin=2)
    return data[:, 0], np.array([pose_matrix(r[1:4], r[4:8]) for r in data])


class GroundTruth:
    def __init__(self, path: str, max_gap: float = 0.1):
        stamps, poses = load_tum(path)
        stamps, first = np.unique(stamps, return_index=True)   # strictly increasing for SLERP
        poses = poses[first]
        self.t, self.max_gap = stamps, max_gap
        self.xyz = poses[:, :3, 3]
        self.slerp = Slerp(stamps, Rotation.from_matrix(poses[:, :3, :3]))

    def pose_at(self, stamp: float) -> Optional[np.ndarray]:
        """World pose at `stamp`, or None if no ground-truth sample is close enough."""
        if stamp < self.t[0] or stamp > self.t[-1]:
            return None
        if np.min(np.abs(self.t - stamp)) > self.max_gap:
            return None
        T = np.eye(4)
        T[:3, 3] = [np.interp(stamp, self.t, self.xyz[:, k]) for k in range(3)]
        T[:3, :3] = self.slerp(stamp).as_matrix()
        return T

    def relative(self, stamp_target: float, stamp_source: float) -> Optional[np.ndarray]:
        """Ground-truth transform mapping source-scan points into the target-scan frame."""
        T_t, T_s = self.pose_at(stamp_target), self.pose_at(stamp_source)
        if T_t is None or T_s is None:
            return None
        return np.linalg.inv(T_t) @ T_s


def pose_error(T_est: np.ndarray, T_gt: np.ndarray):
    """(TE [m], RE [deg]) of one relative pose."""
    T_err = T_gt @ np.linalg.inv(T_est)
    cos_angle = np.clip((np.trace(T_err[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.linalg.norm(T_err[:3, 3])), float(np.degrees(np.arccos(cos_angle)))


KITTI_LENGTHS = (100, 200, 300, 400, 500, 600, 700, 800)   # m
KITTI_STEP = 10                                              # frames between segment starts


def kitti_segments(gt_poses, est_poses, lengths=KITTI_LENGTHS, step=KITTI_STEP) -> np.ndarray:
    """KITTI relative errors of one chain. Returns rows (L, t_err / L, r_err [deg] / L)."""
    xyz = np.array([p[:3, 3] for p in gt_poses])
    dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))])
    rows = []
    for first in range(0, len(gt_poses), step):
        for L in lengths:
            last = np.searchsorted(dist, dist[first] + L)
            if last >= len(gt_poses):
                continue
            d_gt = np.linalg.inv(gt_poses[first]) @ gt_poses[last]
            d_est = np.linalg.inv(est_poses[first]) @ est_poses[last]
            E = np.linalg.inv(d_gt) @ d_est                      # KITTI devkit convention
            angle = np.degrees(np.arccos(np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1)))
            rows.append((L, np.linalg.norm(E[:3, 3]) / L, angle / L))
    return np.array(rows).reshape(-1, 3)


def tum_line(stamp: float, T: np.ndarray) -> str:
    qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
    x, y, z = T[:3, 3]
    return f"{stamp:.9f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"


def relative_pose_line(stamp_target: float, stamp_source: float, T: np.ndarray) -> str:
    """One line of relative_poses.txt: both stamps, then the 12 entries of the top 3x4 of T."""
    values = " ".join(f"{v:.12g}" for v in T[:3, :].ravel())
    return f"{stamp_target:.9f} {stamp_source:.9f} {values}\n"


def read_relative_poses(path: str) -> list:
    """relative_poses.txt -> chains of consecutive pairs, each a list of (stamp_target, stamp_source, T).

    A new chain starts wherever a pair does not continue the previous one, i.e. where a pair is
    missing (e.g. a scan without point cloud, or a partial run).
    """
    chains = []
    for row in np.loadtxt(path, ndmin=2):
        T = np.vstack([row[2:14].reshape(3, 4), [0, 0, 0, 1]])
        if not chains or chains[-1][-1][1] != row[0]:
            chains.append([])
        chains[-1].append((row[0], row[1], T))
    return chains


def chain_stamps(chain) -> list:
    """Frame stamps of a chain: the first target, then every source."""
    return [chain[0][0]] + [stamp_source for _, stamp_source, _ in chain]


def chain_poses(chain, start: Optional[np.ndarray] = None) -> list:
    """Chained poses of a chain, P_{k+1} = P_k T_k, from `start` (identity by default)."""
    poses = [np.eye(4) if start is None else start]
    for _, _, T in chain:
        poses.append(poses[-1] @ T)
    return poses


def scored_chains(chains, gt: GroundTruth):
    """The pieces of the chains that the paper scores: maximal runs of consecutive pairs whose two frames
    both have ground truth. Returns (pieces, number of pairs without ground truth)."""
    pieces, skipped = [], 0
    for chain in chains:
        piece = []
        for pair in chain:
            if gt.relative(pair[0], pair[1]) is None:
                skipped += 1
                if piece:
                    pieces.append(piece)
                piece = []
            else:
                piece.append(pair)
        if piece:
            pieces.append(piece)
    return pieces, skipped


def score_relative_poses(path: str, gt: GroundTruth) -> dict:
    """Score a relative_poses.txt exactly as the paper: TE / RE of every pair whose two frames have ground
    truth (Table 1), and the KITTI drift of the chained estimates, cut wherever a pair is missing or has no
    ground truth (Table 3). t_rel [%] / r_rel [deg/m] are None if no 100 m segment fits."""
    pieces, skipped = scored_chains(read_relative_poses(path), gt)
    te, re, segments = [], [], []
    for piece in pieces:
        for stamp_target, stamp_source, T in piece:
            e_t, e_r = pose_error(T, gt.relative(stamp_target, stamp_source))
            te.append(e_t)
            re.append(e_r)
        segments.append(kitti_segments([gt.pose_at(t) for t in chain_stamps(piece)], chain_poses(piece)))
    seg = np.concatenate(segments) if segments else np.zeros((0, 3))
    return {"te": np.array(te), "re": np.array(re), "skipped": skipped, "chains": len(pieces), "segments": seg,
            "t_rel": 100 * seg[:, 1].mean() if len(seg) else None,
            "r_rel": seg[:, 2].mean() if len(seg) else None}
