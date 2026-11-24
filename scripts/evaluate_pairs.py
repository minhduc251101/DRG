#!/usr/bin/env python3
"""Per-pair translation / rotation error of DRG estimates against ground truth (paper, Sec. 4.1).

Reads relative_poses.txt written by the ROS node or by register_dump.py.

    python scripts/evaluate_pairs.py --est /tmp/drg_loop3/relative_poses.txt \
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

from drg.evaluation import GroundTruth, score_relative_poses  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--est", required=True, help="relative_poses.txt")
    ap.add_argument("--gt", required=True, help="ground truth, TUM format (gt_odom.txt)")
    ap.add_argument("--max-gt-gap", type=float, default=0.1, help="s (paper: 0.1, nyl 0.15)")
    args = ap.parse_args()

    score = score_relative_poses(args.est, GroundTruth(args.gt, args.max_gt_gap))
    te, re = score["te"], score["re"]
    print(f"{len(te)} pairs scored, {score['skipped']} without ground truth within {args.max_gt_gap} s")
    print(f"TE [m]  : mean {te.mean():.4f}  median {np.median(te):.4f}")
    print(f"RE [deg]: mean {re.mean():.4f}  median {np.median(re):.4f}")


if __name__ == "__main__":
    main()
