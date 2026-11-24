#!/usr/bin/env python3
"""4DRadarSLAM keyframe dump (/radar_graph_slam/dump service) -> TUM trajectories for evo.

Each keyframe folder's `data` file holds `stamp`, `estimate` (pose after the g2o pose graph) and `odom_scan2scan`
(the front end's odometry at that keyframe, before the graph). Writes, exactly as the system produced them:

    <out>/stamp_est_traj_<name>.txt        the `estimate` poses (default name: loop_closure)
    <out>/stamp_est_traj_front_end.txt      the `odom_scan2scan` poses

    python3 scripts/slam/dump_to_tum.py --dump dump --out . --name loop_closure      # run with loop closure
    python3 scripts/slam/dump_to_tum.py --dump dump --out . --name no_loop_closure   # run without
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

import numpy as np  # noqa: E402

from drg.evaluation import tum_line  # noqa: E402


def read_dump(dump):
    rows = []
    for name in sorted(os.listdir(dump)):
        path = os.path.join(dump, name, "data")
        if not os.path.isfile(path):
            continue
        lines = [line.split() for line in open(path)]
        stamp, mats = None, {}
        for i, p in enumerate(lines):
            if p and p[0] == "stamp":
                stamp = int(p[1]) + int(p[2]) * 1e-9
            elif p and p[0] in ("estimate", "odom_scan2scan"):
                mats[p[0]] = np.array([[float(x) for x in lines[i + j]] for j in range(1, 5)])
        rows.append((stamp, mats["estimate"], mats.get("odom_scan2scan", mats["estimate"])))
    return sorted(rows, key=lambda r: r[0])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="loop_closure", help="suffix of the after-graph file")
    args = ap.parse_args()
    rows = read_dump(args.dump)
    os.makedirs(args.out, exist_ok=True)
    for name, k in ((args.name, 1), ("front_end", 2)):
        with open(os.path.join(args.out, f"stamp_est_traj_{name}.txt"), "w") as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            f.writelines(tum_line(r[0], r[k]) for r in rows)
    print(f"{len(rows)} keyframes -> {args.out}/stamp_est_traj_{{{args.name},front_end}}.txt")


if __name__ == "__main__":
    main()
