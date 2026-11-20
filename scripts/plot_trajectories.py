#!/usr/bin/env python3
"""Trajectory plots (PDF, top view) of the released results, drawn with evo's plotting module.

    python scripts/plot_trajectories.py [--segmented]

writes, for every sequence,

    results/trajectories/<seq>/trajectory.pdf   DRG odometry over the whole run, never restarted
                                                (stamp_est_traj.txt), SE(3)-aligned to the ground truth
                                                as by `evo_ape -va`, with its ATE RMSE in the legend.
    results/slam_backend/<seq>/trajectory.pdf   DRG front end and DRG + 4DRadarSLAM back end, each
                                                SE(3)-aligned to the ground truth as by `evo_ape -va`,
                                                with its ATE RMSE in the legend.

and with --segmented (after `export_trajectories.py --segmented`)

    results/trajectories/<seq>/trajectory_segmented.pdf
                                                as in Fig. 4 of the paper: each segment starts at the
                                                ground-truth pose of its first scan and is drawn on its own.

Every plot marks each radar dropout: two consecutive scans more than DROPOUT_MIN_S apart while the
ground truth moves more than DROPOUT_MIN_M (loop1: 17.4 s and 90 m, with a 43 deg turn).
"""
import argparse
import os
import sys

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(REPO, "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from evo.core import metrics, sync  # noqa: E402
from evo.core.trajectory import PoseTrajectory3D  # noqa: E402
from evo.tools import file_interface, plot  # noqa: E402

from drg.evaluation import read_relative_poses  # noqa: E402

ODOMETRY_DIR = os.path.join(REPO, "results", "trajectories")
SLAM_DIR = os.path.join(REPO, "results", "slam_backend")
ODOMETRY_SEQUENCES = ("cp", "garden", "nyl", "loop1", "loop2", "loop3")
SLAM_SEQUENCES = ("cp", "garden", "nyl", "loop1", "loop2", "loop3")
XY = plot.PlotMode.xy
DRG_COLOR, FRONT_END_COLOR = "#c1121f", "#e07b00"
DROPOUT_MIN_S, DROPOUT_MIN_M = 2.0, 10.0


def read_tum(path):
    return file_interface.read_tum_trajectory_file(path)


def new_axis(title):
    fig = plt.figure(figsize=(6, 6))
    ax = plot.prepare_axis(fig, XY)
    ax.set_title(title)
    return fig, ax


def length_text(metres):
    return f"{metres:.0f} m" if metres < 1000 else f"{metres / 1000:.1f} km"


def mark_dropouts(ax, gt):
    """Mark every radar dropout (see the module docstring) on the ground truth, with its duration and length."""
    t, xyz = gt.timestamps, gt.positions_xyz
    first = True
    for k in np.where(np.diff(t) > DROPOUT_MIN_S)[0]:
        metres = np.linalg.norm(xyz[k + 1] - xyz[k])
        if metres < DROPOUT_MIN_M:
            continue                                          # a stop, not a dropout
        ax.plot(xyz[k:k + 2, 0], xyz[k:k + 2, 1], "-", color="black", lw=3, zorder=5,
                label="radar dropout" if first else None)
        ax.plot(xyz[k:k + 2, 0], xyz[k:k + 2, 1], "o", color="black", ms=5, zorder=5)
        ax.annotate(f"no radar scan for {t[k + 1] - t[k]:.1f} s, {metres:.0f} m", xy=xyz[k, :2],
                    xytext=(-8, -16), textcoords="offset points", ha="right", fontsize=8, zorder=6)
        first = False
    ax.legend()


def aligned_with_ate(gt, est):
    ref, est = sync.associate_trajectories(gt, est)
    est.align(ref)                                            # SE(3) Umeyama, as evo_ape -va
    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((ref, est))
    return est, ape.get_statistic(metrics.StatisticsType.rmse)


def odometry_figure(seq):
    folder = os.path.join(ODOMETRY_DIR, seq)
    gt = read_tum(os.path.join(folder, "stamp_gt_traj.txt"))
    est, rmse = aligned_with_ate(gt, read_tum(os.path.join(folder, "stamp_est_traj.txt")))
    fig, ax = new_axis(f"{seq}: DRG odometry over the whole run, never restarted")
    plot.traj(ax, XY, gt, "--", "gray", "ground truth")
    plot.traj(ax, XY, est, "-", DRG_COLOR, f"DRG (ATE {rmse:.1f} m)")
    mark_dropouts(ax, gt)
    return fig


def segmented_figure(seq):
    folder = os.path.join(ODOMETRY_DIR, seq)
    gt = read_tum(os.path.join(folder, "stamp_gt_traj.txt"))
    est = read_tum(os.path.join(folder, "stamp_est_traj_segmented.txt"))
    chains = read_relative_poses(os.path.join(REPO, "results", "relative_poses", f"{seq}.txt"))
    sizes = [len(chain) + 1 for chain in chains]          # poses per segment
    assert sum(sizes) == est.num_poses == gt.num_poses, seq

    ends = np.cumsum(sizes)
    xyz = gt.positions_xyz
    length = sum(np.linalg.norm(np.diff(xyz[e - n:e], axis=0), axis=1).sum() for n, e in zip(sizes, ends))
    fig, ax = new_axis(f"{seq}: {length_text(length)}, {len(sizes)} segment{'s' if len(sizes) > 1 else ''}")
    plot.traj(ax, XY, gt, "--", "gray", "ground truth")
    for k, (n, e) in enumerate(zip(sizes, ends)):
        segment = PoseTrajectory3D(poses_se3=est.poses_se3[e - n:e], timestamps=est.timestamps[e - n:e])
        plot.traj(ax, XY, segment, "-", DRG_COLOR, "DRG" if k == 0 else "")
    mark_dropouts(ax, gt)
    return fig


def slam_figure(seq):
    folder = os.path.join(SLAM_DIR, seq)
    gt = read_tum(os.path.join(folder, "stamp_gt_traj.txt"))
    fig, ax = new_axis(f"{seq}: DRG in 4DRadarSLAM, keyframes")
    plot.traj(ax, XY, gt, "--", "gray", "ground truth")
    for name, label, color in (("stamp_est_traj_front_end.txt", "DRG front end", FRONT_END_COLOR),
                               ("stamp_est_traj.txt", "DRG + back end", DRG_COLOR)):
        est, rmse = aligned_with_ate(gt, read_tum(os.path.join(folder, name)))
        plot.traj(ax, XY, est, "-", color, f"{label} (ATE {rmse:.1f} m)")
    mark_dropouts(ax, gt)
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--segmented", action="store_true",
                    help="also plot stamp_est_traj_segmented.txt as trajectory_segmented.pdf (Fig. 4 of the paper)")
    args = ap.parse_args()
    jobs = [(ODOMETRY_DIR, ODOMETRY_SEQUENCES, odometry_figure, "trajectory.pdf"),
            (SLAM_DIR, SLAM_SEQUENCES, slam_figure, "trajectory.pdf")]
    if args.segmented:
        jobs.append((ODOMETRY_DIR, ODOMETRY_SEQUENCES, segmented_figure, "trajectory_segmented.pdf"))
    for folder, sequences, figure, name in jobs:
        for seq in sequences:
            path = os.path.join(folder, seq, name)
            fig = figure(seq)
            fig.savefig(path, bbox_inches="tight", metadata={"CreationDate": None})  # byte-stable rebuilds
            plt.close(fig)
            print(f"wrote {os.path.relpath(path, REPO)}")


if __name__ == "__main__":
    main()
