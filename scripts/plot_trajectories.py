#!/usr/bin/env python3
"""Trajectory plots (PDF, top view) of the released results, drawn with evo's plotting module.

    python scripts/plot_trajectories.py

writes, for every sequence,

    results/trajectories/<seq>/trajectory.pdf   DRG odometry and the ground truth. Each segment starts at
                                                the ground-truth pose of its first scan and is drawn on
                                                its own, as in Fig. 4 of the paper.
    results/slam_backend/<seq>/trajectory.pdf   DRG front end and DRG + 4DRadarSLAM back end, each
                                                SE(3)-aligned to the ground truth as by `evo_ape -va`,
                                                with its ATE RMSE in the legend.
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
SLAM_SEQUENCES = ("garden", "nyl", "loop1", "loop2", "loop3")
XY = plot.PlotMode.xy
DRG_COLOR, FRONT_END_COLOR = "#c1121f", "#e07b00"


def read_tum(path):
    return file_interface.read_tum_trajectory_file(path)


def new_axis(title):
    fig = plt.figure(figsize=(6, 6))
    ax = plot.prepare_axis(fig, XY)
    ax.set_title(title)
    return fig, ax


def length_text(metres):
    return f"{metres:.0f} m" if metres < 1000 else f"{metres / 1000:.1f} km"


def odometry_figure(seq):
    folder = os.path.join(ODOMETRY_DIR, seq)
    gt, est = read_tum(os.path.join(folder, "stamp_gt_traj.txt")), read_tum(os.path.join(folder, "stamp_est_traj.txt"))
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
    return fig


def slam_figure(seq):
    folder = os.path.join(SLAM_DIR, seq)
    gt = read_tum(os.path.join(folder, "stamp_gt_traj.txt"))
    fig, ax = new_axis(f"{seq}: DRG in 4DRadarSLAM, keyframes")
    plot.traj(ax, XY, gt, "--", "gray", "ground truth")
    for name, label, color in (("stamp_est_traj_front_end.txt", "DRG front end", FRONT_END_COLOR),
                               ("stamp_est_traj.txt", "DRG + back end", DRG_COLOR)):
        ref, est = sync.associate_trajectories(gt, read_tum(os.path.join(folder, name)))
        est.align(ref)                                      # SE(3) Umeyama, as evo_ape -va
        ape = metrics.APE(metrics.PoseRelation.translation_part)
        ape.process_data((ref, est))
        rmse = ape.get_statistic(metrics.StatisticsType.rmse)
        plot.traj(ax, XY, est, "-", color, f"{label} (ATE {rmse:.1f} m)")
    return fig


def main():
    argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter).parse_args()
    for folder, sequences, figure in ((ODOMETRY_DIR, ODOMETRY_SEQUENCES, odometry_figure),
                                      (SLAM_DIR, SLAM_SEQUENCES, slam_figure)):
        for seq in sequences:
            path = os.path.join(folder, seq, "trajectory.pdf")
            fig = figure(seq)
            fig.savefig(path, bbox_inches="tight")
            plt.close(fig)
            print(f"wrote {os.path.relpath(path, REPO)}")


if __name__ == "__main__":
    main()
