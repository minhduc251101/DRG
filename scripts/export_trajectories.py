#!/usr/bin/env python3
"""Trajectories of the released DRG estimates for evo (TUM format), one folder per sequence.

The per-pair estimates in results/relative_poses/<seq>.txt are chained, P_{k+1} = P_k T_k, with no
keyframing, smoothing or loop closure. These are the trajectories behind the paper's t_rel / r_rel
(Table 3). A chain is cut where a pair is missing (a frame without ground truth). Each chain starts
at the ground-truth pose of its first scan, so every pose is in the ground-truth world frame.
Writes, for each sequence:

    results/trajectories/<seq>/stamp_est_traj.txt   DRG
    results/trajectories/<seq>/stamp_gt_traj.txt    NTU4DRadLM ground truth at the same stamps
                                                    (linear / SLERP interpolation, as in the paper)

    python scripts/export_trajectories.py --datasets config/datasets.yaml
    evo_traj tum results/trajectories/loop3/stamp_est_traj.txt --ref results/trajectories/loop3/stamp_gt_traj.txt --plot

config/datasets.yaml gives the ground truth (gt_odom.txt) and its gap limit for each sequence; see
config/datasets_example.yaml. The dump paths in that file are not used here.
"""
import argparse
import os
import sys

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(REPO, "src"))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from drg.evaluation import GroundTruth, chain_poses, chain_stamps, read_relative_poses, tum_line  # noqa: E402

SEQUENCES = ("cp", "garden", "nyl", "loop1", "loop2", "loop3")


def write_tum(path, header, stamps, poses):
    with open(path, "w") as f:
        f.write(f"# {header}\n# timestamp tx ty tz qx qy qz qw\n")
        f.writelines(tum_line(t, T) for t, T in zip(stamps, poses))


def export_sequence(seq, relative_poses_path, gt, out_dir):
    """Write the two TUM files of one sequence. Returns (poses, chains, path length [m])."""
    chains = read_relative_poses(relative_poses_path)
    stamps, est_poses, gt_poses, length = [], [], [], 0.0
    for chain in chains:
        chain_gt = [gt.pose_at(t) for t in chain_stamps(chain)]
        if any(p is None for p in chain_gt):
            sys.exit(f"{seq}: a chained frame has no ground truth within the gap limit")
        stamps += chain_stamps(chain)
        gt_poses += chain_gt
        est_poses += chain_poses(chain, start=chain_gt[0])
        xyz = np.array([p[:3, 3] for p in chain_gt])
        length += np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()

    os.makedirs(out_dir, exist_ok=True)
    write_tum(os.path.join(out_dir, "stamp_est_traj.txt"),
              f"DRG on NTU4DRadLM {seq}: chained scan-to-scan estimates, {len(chains)} chain(s), "
              "each starting at the ground-truth pose of its first scan",
              stamps, est_poses)
    write_tum(os.path.join(out_dir, "stamp_gt_traj.txt"),
              f"NTU4DRadLM {seq} ground truth (gt_odom.txt) interpolated at the radar stamps",
              stamps, gt_poses)
    return len(stamps), len(chains), length


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--datasets", required=True, help="YAML with gt and max_gt_gap per sequence")
    ap.add_argument("--sequences", nargs="+", default=SEQUENCES, choices=SEQUENCES)
    ap.add_argument("--est-dir", default=os.path.join(REPO, "results", "relative_poses"))
    ap.add_argument("--out-dir", default=os.path.join(REPO, "results", "trajectories"))
    args = ap.parse_args()

    with open(args.datasets) as f:
        datasets = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(args.datasets))

    print(f"{'sequence':9s} {'poses':>6s} {'chains':>6s} {'length [m]':>10s}")
    for seq in args.sequences:
        cfg = datasets[seq]
        gt = GroundTruth(os.path.join(base, cfg["gt"]), cfg.get("max_gt_gap", 0.1))
        n_poses, n_chains, length = export_sequence(
            seq, os.path.join(args.est_dir, f"{seq}.txt"), gt, os.path.join(args.out_dir, seq))
        print(f"{seq:9s} {n_poses:6d} {n_chains:6d} {length:10.1f}")


if __name__ == "__main__":
    main()
