#!/usr/bin/env python3
"""Register consecutive scans of a 4DRadarSLAM dump with DRG - the protocol of the paper.

Pairs are (k, k+1) by frame-folder number, from --start for --num-pairs pairs (default: all).
Outputs, in --out:
    drg_pairs.json        per-pair estimate and diagnostics (+ TE / RE if --gt is given)
    relative_poses.txt    t_target t_source and the top 3x4 of T, one pair per line
    stamp_est_traj.txt    chained trajectory P_{k+1} = P_k T_k in TUM format (for evo)

    python scripts/register_dump.py --dump <dump_root> --stats config/norm_stats/cp.yaml \
        --gt <gt_odom.txt> --start 4470 --num-pairs 30 --out results/cp_set1
"""
import argparse
import json
import os
import sys

# Single-threaded, as for the paper: with more threads (BLAS / OpenMP) the estimates change in
# the last digits (~1e-11 m). This must run before numpy and open3d are imported.
os.environ["OMP_NUM_THREADS"] = "1"

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import numpy as np  # noqa: E402

from drg import DRG, DRGParams, NormStats  # noqa: E402
from drg.evaluation import GroundTruth, relative_pose_line, tum_line  # noqa: E402
from drg.io import DumpSequence  # noqa: E402
from drg.sequence import register_pairs  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True, help="dump root with %%06d/{cloud.pcd,data}")
    ap.add_argument("--stats", required=True, help="norm-stats YAML (scripts/compute_norm_stats.py)")
    ap.add_argument("--params", default=None, help="DRG parameter YAML (default: paper values)")
    ap.add_argument("--gt", default=None, help="ground truth in TUM format, to report TE / RE")
    ap.add_argument("--max-gt-gap", type=float, default=0.1, help="s (paper: 0.1, nyl 0.15)")
    ap.add_argument("--start", type=int, default=None, help="first target frame (default: first)")
    ap.add_argument("--num-pairs", type=int, default=None, help="default: to the end")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    params = DRGParams.from_yaml(args.params) if args.params else DRGParams()
    drg = DRG(params, NormStats.from_yaml(args.stats))
    seq = DumpSequence(args.dump)
    gt = GroundTruth(args.gt, args.max_gt_gap) if args.gt else None

    start = seq.indices()[0] if args.start is None else args.start
    last = seq.indices()[-1]
    n_pairs = last - start if args.num_pairs is None else min(args.num_pairs, last - start)
    pairs = [(start + n, start + n + 1) for n in range(n_pairs)]

    os.makedirs(args.out, exist_ok=True)
    rows, pose = [], np.eye(4)
    with open(os.path.join(args.out, "relative_poses.txt"), "w") as rel_f, \
         open(os.path.join(args.out, "stamp_est_traj.txt"), "w") as traj_f:
        for n, row in enumerate(register_pairs(drg, seq, pairs, gt)):
            if n == 0:
                traj_f.write(tum_line(row["stamp_target"], pose))
            pose = pose @ row["T"]
            traj_f.write(tum_line(row["stamp_source"], pose))
            rel_f.write(relative_pose_line(row["stamp_target"], row["stamp_source"], row["T"]))

            msg = (f"[{n + 1}/{n_pairs}] {row['target']}->{row['source']} gated "
                   f"{row['n_gated']}/{row['n_putative']} w={row['fusion_weight']:.2f}")
            if "te" in row:
                msg += f" | TE {row['te']:.4f} m RE {row['re']:.3f} deg"
            print(msg + f" ({row['time_s']:.1f} s)", flush=True)
            row["T"] = row["T"].tolist()
            rows.append(row)

    summary = {"pairs": len(rows)}
    scored = [r for r in rows if "te" in r]
    if scored:
        te, re = np.array([r["te"] for r in scored]), np.array([r["re"] for r in scored])
        summary.update(scored=len(scored), te_mean=te.mean(), te_median=float(np.median(te)),
                       re_mean=re.mean(), re_median=float(np.median(re)))
        print(f"\n{len(scored)} scored pairs: TE mean {te.mean():.4f} m (median {np.median(te):.4f}), "
              f"RE mean {re.mean():.3f} deg (median {np.median(re):.3f})")
    with open(os.path.join(args.out, "drg_pairs.json"), "w") as f:
        json.dump({"dump": args.dump, "stats": args.stats, "params": vars(params),
                   "summary": summary, "pairs": rows}, f, indent=1)


if __name__ == "__main__":
    main()
