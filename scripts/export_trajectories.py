#!/usr/bin/env python3
"""Trajectories of the released DRG estimates for evo (TUM format), one folder per sequence.

The per-pair estimates in results/relative_poses/<seq>.txt are chained over the whole run,
P_{k+1} = P_k T_k, from the ground-truth pose of the first scan, with no keyframing, smoothing, loop
closure or restart. The few pairs the paper left out for lack of ground truth are bridged at constant
velocity (the last estimated translation per second, in the body frame, no rotation), exactly as in
the odometry DRG feeds to the SLAM back end (results/slam_backend). Writes, for each sequence:

    results/trajectories/<seq>/stamp_est_traj.txt   DRG over the whole run
    results/trajectories/<seq>/stamp_gt_traj.txt    NTU4DRadLM ground truth at the same stamps
                                                    (linear / SLERP interpolation, as in the paper)

With --segmented it also writes the version drawn in Fig. 4 of the paper, in which the chain is cut
at each missing pair and every segment starts at the ground-truth pose of its first scan:

    results/trajectories/<seq>/stamp_est_traj_segmented.txt

    python scripts/export_trajectories.py --datasets config/datasets.yaml
    evo_ape tum results/trajectories/loop3/stamp_gt_traj.txt results/trajectories/loop3/stamp_est_traj.txt -va --plot

config/datasets.yaml gives the ground truth (gt_odom.txt) and its gap limit for each sequence; see
config/datasets_example.yaml. The dump paths in that file are not used here. The paper's t_rel and
r_rel (Table 3) are computed from results/relative_poses/ by scripts/evaluate_trajectory.py, which
scores each segment on its own; they do not depend on these files.
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


def whole_run_poses(chains, start):
    """Chain every pair from `start` without restarting; bridge each missing pair at constant velocity.

    Inside a chain P_{k+1} = P_k T_k. Between two chains (pairs left out for lack of ground truth)
    the pose is carried forward by the last estimated translation per second over the gap, in the
    body frame, with no rotation, exactly as the SLAM back end's odometry table does.
    """
    poses, v = [start], np.zeros(3)
    for i, chain in enumerate(chains):
        if i > 0:
            bridge = np.eye(4)
            bridge[:3, 3] = v * (chain[0][0] - chains[i - 1][-1][1])
            poses.append(poses[-1] @ bridge)
        for stamp_target, stamp_source, T in chain:
            poses.append(poses[-1] @ T)
            v = T[:3, 3] / max(stamp_source - stamp_target, 1e-6)
    return poses


def export_sequence(seq, relative_poses_path, gt, out_dir, segmented=False):
    """Write the TUM files of one sequence. Returns (poses, bridged gaps, ground-truth path length [m])."""
    chains = read_relative_poses(relative_poses_path)
    stamps, gt_poses, segmented_poses = [], [], []
    for chain in chains:
        chain_gt = [gt.pose_at(t) for t in chain_stamps(chain)]
        if any(p is None for p in chain_gt):
            sys.exit(f"{seq}: a chained frame has no ground truth within the gap limit")
        stamps += chain_stamps(chain)
        gt_poses += chain_gt
        segmented_poses += chain_poses(chain, start=chain_gt[0])
    whole_run = whole_run_poses(chains, start=gt_poses[0])
    assert len(whole_run) == len(segmented_poses) == len(stamps), seq
    xyz = np.array([p[:3, 3] for p in gt_poses])
    length = np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()

    os.makedirs(out_dir, exist_ok=True)
    write_tum(os.path.join(out_dir, "stamp_est_traj.txt"),
              f"DRG on NTU4DRadLM {seq}: per-pair estimates chained over the whole run from the ground-truth "
              f"pose of the first scan, never restarted; {len(chains) - 1} missing pair(s) bridged at constant velocity",
              stamps, whole_run)
    write_tum(os.path.join(out_dir, "stamp_gt_traj.txt"),
              f"NTU4DRadLM {seq} ground truth (gt_odom.txt) interpolated at the radar stamps",
              stamps, gt_poses)
    if segmented:
        write_tum(os.path.join(out_dir, "stamp_est_traj_segmented.txt"),
                  f"DRG on NTU4DRadLM {seq}: chained scan-to-scan estimates, {len(chains)} segment(s), "
                  "each starting at the ground-truth pose of its first scan (Fig. 4 of the paper)",
                  stamps, segmented_poses)
    return len(stamps), len(chains) - 1, length


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--datasets", required=True, help="YAML with gt and max_gt_gap per sequence")
    ap.add_argument("--sequences", nargs="+", default=SEQUENCES, choices=SEQUENCES)
    ap.add_argument("--est-dir", default=os.path.join(REPO, "results", "relative_poses"))
    ap.add_argument("--out-dir", default=os.path.join(REPO, "results", "trajectories"))
    ap.add_argument("--segmented", action="store_true",
                    help="also write stamp_est_traj_segmented.txt, restarted at the ground truth after each "
                         "missing pair (Fig. 4 of the paper)")
    args = ap.parse_args()

    with open(args.datasets) as f:
        datasets = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(args.datasets))

    print(f"{'sequence':9s} {'poses':>6s} {'bridged':>7s} {'length [m]':>10s}")
    for seq in args.sequences:
        cfg = datasets[seq]
        gt = GroundTruth(os.path.join(base, cfg["gt"]), cfg.get("max_gt_gap", 0.1))
        n_poses, n_bridged, length = export_sequence(
            seq, os.path.join(args.est_dir, f"{seq}.txt"), gt, os.path.join(args.out_dir, seq), args.segmented)
        print(f"{seq:9s} {n_poses:6d} {n_bridged:7d} {length:10.1f}")


if __name__ == "__main__":
    main()
