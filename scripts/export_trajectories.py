#!/usr/bin/env python3
"""Trajectories of the released DRG estimates for evo (TUM format), one folder per sequence.

results/relative_poses/<seq>.txt holds DRG's estimate for every consecutive scan pair of the run. They
are chained over the whole run, P_{k+1} = P_k T_k, from the identity, with no ground truth, keyframing,
smoothing, loop closure or restart: the trajectory DRG itself outputs. Writes, for each sequence:

    results/trajectories/<seq>/stamp_est_traj.txt   DRG odometry over the whole run
    results/trajectories/<seq>/stamp_gt_traj.txt    NTU4DRadLM ground truth at the same stamps (linear /
                                                    SLERP interpolation, as in the paper); stamps with no
                                                    ground-truth sample within the gap limit are left out

With --segmented it also writes the version drawn in Fig. 4 of the paper. It is NOT a trajectory DRG
outputs: only the pairs the paper scored are used (both frames with ground truth), the chain is cut at
every other pair, and each piece starts at the ground-truth pose of its first scan:

    results/trajectories/<seq>/stamp_est_traj_segmented.txt

    python scripts/export_trajectories.py --datasets config/datasets.yaml
    cd results/trajectories/loop3 && evo_ape tum stamp_gt_traj.txt stamp_est_traj.txt -va --plot

config/datasets.yaml gives the ground truth (gt_odom.txt) and its gap limit for each sequence; see
config/datasets_example.yaml. The dump paths in that file are not used here. The paper's t_rel and
r_rel (Table 3) are computed from results/relative_poses/ by scripts/evaluate_trajectory.py.

Where a scan of the dump has no point cloud (cp: frames 3247 and 3312), the pairs (k, k+1) touching it
do not exist; the chain crosses it with DRG's estimate of the pair that spans it, read from
results/relative_poses/<seq>_across_missing_scans.txt. If a pair is missing for any other reason (e.g.
a partial run), the step is bridged at constant velocity (the last estimated translation per second,
body frame, no rotation) and the number of bridged steps is printed. In the released files every step
is a DRG estimate: nothing is bridged.
"""
import argparse
import os
import sys

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(REPO, "src"))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from drg.evaluation import (GroundTruth, chain_poses, chain_stamps, read_relative_poses,  # noqa: E402
                            scored_chains, tum_line)

SEQUENCES = ("cp", "garden", "nyl", "loop1", "loop2", "loop3", "garden_every_scan")


def write_tum(path, header, stamps, poses):
    with open(path, "w") as f:
        f.writelines(f"# {line}\n" for line in header)
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        f.writelines(tum_line(t, T) for t, T in zip(stamps, poses))


def whole_run(chains, across=None):
    """Chain every pair from the identity. Between two chains use the pair that spans the gap if `across`
    has it, else bridge at constant velocity (see the docstring). Returns stamps, poses, #across, #bridged."""
    across = across or {}
    stamps, poses, v, n_across, n_bridged = [chains[0][0][0]], [np.eye(4)], np.zeros(3), 0, 0
    for i, chain in enumerate(chains):
        if i > 0:
            gap = (chains[i - 1][-1][1], chain[0][0])
            if gap in across:
                step = across[gap]
                n_across += 1
            else:
                step = np.eye(4)
                step[:3, 3] = v * (gap[1] - gap[0])
                n_bridged += 1
            stamps.append(chain[0][0])
            poses.append(poses[-1] @ step)
        for stamp_target, stamp_source, T in chain:
            stamps.append(stamp_source)
            poses.append(poses[-1] @ T)
            v = T[:3, 3] / max(stamp_source - stamp_target, 1e-6)
    return stamps, poses, n_across, n_bridged


def export_sequence(seq, relative_poses_path, gt, out_dir, segmented=False):
    """Write the TUM files of one sequence. Returns a dict of counts."""
    chains = read_relative_poses(relative_poses_path)
    across_path = relative_poses_path[:-len(".txt")] + "_across_missing_scans.txt"
    across = ({(t, s): T for chain in read_relative_poses(across_path) for t, s, T in chain}
              if os.path.exists(across_path) else {})
    stamps, poses, n_across, n_bridged = whole_run(chains, across)
    n_pairs = sum(len(c) for c in chains) + n_across
    gt_stamps, gt_poses = [], []
    for t in stamps:
        p = gt.pose_at(t)
        if p is not None:
            gt_stamps.append(t)
            gt_poses.append(p)
    xyz = np.array([p[:3, 3] for p in gt_poses])
    info = {"poses": len(stamps), "pairs": n_pairs, "across": n_across, "bridged": n_bridged,
            "without_gt": len(stamps) - len(gt_stamps),
            "length": float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum())}

    os.makedirs(out_dir, exist_ok=True)
    notes = []
    if n_across:
        notes.append(f"{n_across} of the pairs span a scan of the dump without point cloud")
    if n_bridged:
        notes.append(f"{n_bridged} missing pair(s) bridged at constant velocity")
    write_tum(os.path.join(out_dir, "stamp_est_traj.txt"),
              [f"DRG odometry on NTU4DRadLM {seq}: DRG's estimates of all {n_pairs} pairs of consecutive scans chained",
               "over the whole run from the identity; no ground truth, restart, keyframing, smoothing or loop closure"
               + (" (" + "; ".join(notes) + ")" if notes else "")],
              stamps, poses)
    write_tum(os.path.join(out_dir, "stamp_gt_traj.txt"),
              [f"NTU4DRadLM {seq} ground truth (gt_odom.txt) interpolated at the radar stamps (linear / SLERP);",
               f"{info['without_gt']} of {len(stamps)} stamps have no ground-truth sample within {gt.max_gap} s "
               "and are left out"],
              gt_stamps, gt_poses)
    if segmented:
        pieces, _ = scored_chains(chains, gt)
        seg_stamps, seg_poses = [], []
        for piece in pieces:
            piece_stamps = chain_stamps(piece)
            seg_stamps += piece_stamps
            seg_poses += chain_poses(piece, start=gt.pose_at(piece_stamps[0]))
        write_tum(os.path.join(out_dir, "stamp_est_traj_segmented.txt"),
                  [f"Fig. 4 of the paper, NTU4DRadLM {seq}. NOT a trajectory DRG outputs: only the pairs with ground",
                   f"truth, cut into {len(pieces)} segment(s) at the others, each starting at the GROUND-TRUTH pose of its first scan"],
                  seg_stamps, seg_poses)
        info["segments"] = len(pieces)
    return info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--datasets", required=True, help="YAML with gt and max_gt_gap per sequence")
    ap.add_argument("--sequences", nargs="+", default=SEQUENCES, choices=SEQUENCES)
    ap.add_argument("--est-dir", default=os.path.join(REPO, "results", "relative_poses"))
    ap.add_argument("--out-dir", default=os.path.join(REPO, "results", "trajectories"))
    ap.add_argument("--segmented", action="store_true",
                    help="also write stamp_est_traj_segmented.txt, restarted at the ground truth after each "
                         "pair without ground truth (Fig. 4 of the paper)")
    args = ap.parse_args()

    with open(args.datasets) as f:
        datasets = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(args.datasets))

    print(f"{'sequence':18s} {'poses':>6s} {'no GT':>6s} {'across':>6s} {'bridged':>7s} {'length [m]':>10s}")
    for seq in args.sequences:
        cfg = datasets[seq]
        gt = GroundTruth(os.path.join(base, cfg["gt"]), cfg.get("max_gt_gap", 0.1))
        info = export_sequence(seq, os.path.join(args.est_dir, f"{seq}.txt"), gt,
                               os.path.join(args.out_dir, seq), args.segmented and seq != "garden_every_scan")
        print(f"{seq:18s} {info['poses']:6d} {info['without_gt']:6d} {info['across']:6d} {info['bridged']:7d} "
              f"{info['length']:10.1f}")


if __name__ == "__main__":
    main()
