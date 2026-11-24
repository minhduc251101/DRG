#!/usr/bin/env python3
"""DRG odometry table for the 4DRadarSLAM back end (results/slam_backend), from the released estimates.

One line per scan: "stamp tx ty tz qx qy qz qw vx vy vz 0", P_0 = identity, P_{k+1} = P_k T_k with T_k DRG's
estimate of the pair (k, k+1) (twist = DRG's translation / dt). Where a scan of the dump has no point cloud (cp), the
pair that spans it is taken from <seq>_across_missing_scans.txt, so every step is a DRG estimate. No ground truth.

    python3 scripts/slam/build_odom_table.py --relative-poses results/relative_poses/loop3.txt --out drg_odom_loop3.txt
    # garden: the back end runs on every scan, i.e. results/relative_poses/garden_every_scan.txt
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

import numpy as np  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--relative-poses", required=True, help="results/relative_poses/<seq>.txt")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    rows = np.loadtxt(args.relative_poses, ndmin=2)
    across = args.relative_poses[:-len(".txt")] + "_across_missing_scans.txt"
    if os.path.exists(across):
        rows = np.concatenate([rows, np.loadtxt(across, ndmin=2)])
    rows = rows[np.argsort(rows[:, 0])]
    if not np.all(rows[1:, 0] == rows[:-1, 1]):
        sys.exit("the pairs do not form one chain (a pair is missing)")

    def line(t, P, v):
        q = Rotation.from_matrix(P[:3, :3]).as_quat()
        return "%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f %.6f %.6f %.6f 0" % (
            t, P[0, 3], P[1, 3], P[2, 3], q[0], q[1], q[2], q[3], v[0], v[1], v[2])

    P, v = np.eye(4), np.zeros(3)
    lines = [line(rows[0, 0], P, v)]
    for r in rows:
        T = np.vstack([r[2:14].reshape(3, 4), [0, 0, 0, 1]])
        P = P @ T
        v = T[:3, 3] / max(r[1] - r[0], 1e-6)
        lines.append(line(r[1], P, v))
    with open(args.out, "w") as f:
        f.write(f"# DRG odometry from {os.path.basename(args.relative_poses)}: {len(lines)} scans, {len(rows)} DRG pairs\n"
                "# stamp tx ty tz qx qy qz qw vx vy vz extrapolated\n")
        f.write("\n".join(lines) + "\n")
    print(f"{len(lines)} scans, {len(rows)} DRG pairs -> {args.out}")


if __name__ == "__main__":
    main()
