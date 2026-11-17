#!/usr/bin/env python3
"""KITTI relative drift of the chained DRG estimates (paper, Table 3).

The per-pair estimates are chained, P_{k+1} = P_k T_k, with no keyframing, smoothing or loop
closure. A chain is cut wherever a pair is missing. For every 10th frame of a chain and every
segment length L in 100, 200, ..., 800 m (along the ground-truth path), the end-point error of the
segment gives t_err / L and r_err / L; t_rel [%] and r_rel [deg/m] are their means over all
segments (KITTI odometry devkit).

    python scripts/evaluate_trajectory.py --est results/relative_poses/loop3.txt \
        --gt /data/NTU4DRadLM/loop3/gt_odom.txt
"""
import argparse
import os
import sys

# Single-threaded, as for the paper: with more threads (BLAS / OpenMP) the estimates change in
# the last digits (~1e-11 m). This must run before numpy and open3d are imported.
os.environ["OMP_NUM_THREADS"] = "1"

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import numpy as np  # noqa: E402

from drg.evaluation import GroundTruth, chain_poses, chain_stamps, kitti_segments, read_relative_poses  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--est", required=True, help="relative_poses.txt")
    ap.add_argument("--gt", required=True, help="ground truth, TUM format (gt_odom.txt)")
    ap.add_argument("--max-gt-gap", type=float, default=0.1, help="s (paper: 0.1, nyl 0.15)")
    args = ap.parse_args()

    gt = GroundTruth(args.gt, args.max_gt_gap)
    segments, n_chains = [], 0
    for chain in read_relative_poses(args.est):
        gt_poses = [gt.pose_at(t) for t in chain_stamps(chain)]
        if any(p is None for p in gt_poses):
            sys.exit("a chained frame has no ground truth within --max-gt-gap")
        segments.append(kitti_segments(gt_poses, chain_poses(chain)))
        n_chains += 1

    seg = np.concatenate([s for s in segments if len(s)])
    lengths = sorted({int(L) for L in seg[:, 0]})
    print(f"{n_chains} chain(s), {len(seg)} segments of {lengths} m")
    print(f"t_rel = {100 * seg[:, 1].mean():.2f} %   r_rel = {seg[:, 2].mean():.4f} deg/m")


if __name__ == "__main__":
    main()
