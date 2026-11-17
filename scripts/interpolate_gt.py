#!/usr/bin/env python3
"""Write the ground truth at the stamps of an estimated trajectory, for evo.

The NTU4DRadLM ground truth is sampled at 10 Hz and the radar at 12 Hz. Matching each radar pose
to the nearest ground-truth sample (evo's --t_max_diff) is up to 50 ms off, i.e. up to 0.4 m at
the 7-8 m/s of the loop sequences. This script interpolates the ground truth at every stamp of
the estimate instead (linear in position, SLERP in rotation; the same interpolation as the
paper's per-pair errors), so evo compares poses taken at the same instant:

    python scripts/interpolate_gt.py --gt <gt_odom.txt> --est stamp_est_traj.txt --out stamp_gt_traj.txt
    evo_ape tum stamp_gt_traj.txt stamp_est_traj.txt -va --plot
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import numpy as np  # noqa: E402

from drg.evaluation import GroundTruth, tum_line  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt", required=True, help="ground truth, TUM format (gt_odom.txt)")
    ap.add_argument("--est", required=True, help="estimated trajectory, TUM format (stamp_est_traj.txt)")
    ap.add_argument("--out", required=True, help="output: ground truth at the estimate's stamps, TUM")
    ap.add_argument("--max-gt-gap", type=float, default=0.1, help="s (paper: 0.1, nyl 0.15)")
    args = ap.parse_args()

    gt = GroundTruth(args.gt, args.max_gt_gap)
    stamps = np.loadtxt(args.est, comments="#", ndmin=2)[:, 0]
    written = 0
    with open(args.out, "w") as f:
        for stamp in stamps:
            pose = gt.pose_at(stamp)
            if pose is not None:
                f.write(tum_line(stamp, pose))
                written += 1
    print(f"wrote {args.out}: {written} of {len(stamps)} stamps "
          f"({len(stamps) - written} without ground truth within {args.max_gt_gap} s)")


if __name__ == "__main__":
    main()
