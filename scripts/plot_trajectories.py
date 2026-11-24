#!/usr/bin/env python3
"""Trajectory plots (PDF, top view) of the released results, drawn with evo's plotting module.

    python scripts/plot_trajectories.py [--segmented]

writes, for every sequence,

    results/trajectories/<seq>/trajectory.pdf   DRG odometry over the whole run, never restarted
                                                (stamp_est_traj.txt), SE(3)-aligned to the ground truth
                                                as by `evo_ape -va`, with its ATE RMSE in the legend.
    results/slam_backend/<seq>/trajectory.pdf   DRG front end, and 4DRadarSLAM's back end on it without
                                                and with loop closure, each SE(3)-aligned to the ground
                                                truth as by `evo_ape -va`, with its ATE RMSE in the legend.

and with --segmented (after `export_trajectories.py --segmented`)

    results/trajectories/<seq>/trajectory_segmented.pdf
                                                as in Fig. 4 of the paper: each segment starts at the
                                                ground-truth pose of its first scan and is drawn on its own.

and with --overview the two figures of the README:

    docs/figures/trajectories_real.png          nyl, loop1, loop2, loop3: DRG odometry and 4DRadarSLAM on
                                                it without / with loop closure, each aligned as by evo_ape -va
    docs/figures/loop1_loop2_paper_vs_real.png  loop1 and loop2 as drawn in Fig. 4 of the paper (restarted
                                                at the ground truth) and as DRG really runs (never restarted)

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

ODOMETRY_DIR = os.path.join(REPO, "results", "trajectories")
SLAM_DIR = os.path.join(REPO, "results", "slam_backend")
ODOMETRY_SEQUENCES = ("cp", "garden", "nyl", "loop1", "loop2", "loop3", "garden_every_scan")
SLAM_SEQUENCES = ("cp", "garden", "nyl", "loop1", "loop2", "loop3")
XY = plot.PlotMode.xy
DRG_COLOR, FRONT_END_COLOR, NO_LOOP_COLOR = "#c1121f", "#e07b00", "#1d4ed8"
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
    plot.traj(ax, XY, est, "-", DRG_COLOR, f"DRG odometry (ATE {rmse:.1f} m)")
    mark_dropouts(ax, gt)
    return fig


def segment_starts(seq, segmented):
    """Indices where a segment of the segmented (Fig. 4) trajectory starts, plus its length at the end: a new
    segment starts wherever two consecutive poses are not a pair of results/relative_poses/<seq>.txt."""
    rel = np.loadtxt(os.path.join(REPO, "results", "relative_poses", f"{seq}.txt"), ndmin=2)
    pairs = {(round(a, 6), round(b, 6)) for a, b in rel[:, :2]}
    t = segmented.timestamps
    cut = [k + 1 for k in range(len(t) - 1) if (round(t[k], 6), round(t[k + 1], 6)) not in pairs]
    return np.array([0] + cut + [len(t)])


def segmented_figure(seq):
    folder = os.path.join(ODOMETRY_DIR, seq)
    gt = read_tum(os.path.join(folder, "stamp_gt_traj.txt"))
    est = read_tum(os.path.join(folder, "stamp_est_traj_segmented.txt"))
    starts = segment_starts(seq, est)

    xyz = est.positions_xyz
    fig, ax = new_axis(f"{seq}: Fig. 4 of the paper, {len(starts) - 1} segment{'s' if len(starts) > 2 else ''} "
                       "restarted at the ground truth")
    plot.traj(ax, XY, gt, "--", "gray", "ground truth")
    for k, (a, b) in enumerate(zip(starts[:-1], starts[1:])):
        segment = PoseTrajectory3D(poses_se3=est.poses_se3[a:b], timestamps=est.timestamps[a:b])
        plot.traj(ax, XY, segment, "-", DRG_COLOR, "DRG, restarted at the ground truth" if k == 0 else "")
        ax.plot(xyz[a, 0], xyz[a, 1], "o", color=DRG_COLOR, ms=3)
    mark_dropouts(ax, gt)
    return fig


def slam_ground_truth(folder, estimate_name, default):
    """Ground truth at the keyframes of that estimate's run (stamp_gt_traj_<name>.txt if the runs differ)."""
    path = os.path.join(folder, estimate_name.replace("stamp_est_traj", "stamp_gt_traj"))
    return read_tum(path) if os.path.exists(path) else default


def slam_figure(seq):
    folder = os.path.join(SLAM_DIR, seq)
    gt = read_tum(os.path.join(folder, "stamp_gt_traj.txt"))
    fig, ax = new_axis(f"{seq}: DRG in 4DRadarSLAM, keyframes")
    plot.traj(ax, XY, gt, "--", "gray", "ground truth")
    for name, label, color in (("stamp_est_traj_front_end.txt", "DRG odometry", FRONT_END_COLOR),
                               ("stamp_est_traj_no_loop_closure.txt", "+ back end, no loop closure", NO_LOOP_COLOR),
                               ("stamp_est_traj_loop_closure.txt", "+ back end, loop closure", DRG_COLOR)):
        est, rmse = aligned_with_ate(slam_ground_truth(folder, name, gt), read_tum(os.path.join(folder, name)))
        plot.traj(ax, XY, est, "-", color, f"{label} (ATE {rmse:.1f} m)")
    mark_dropouts(ax, gt)
    return fig


def panel_slam(ax, seq):
    """One README panel: ground truth and the three SLAM-folder trajectories, each aligned as by evo_ape -va."""
    folder = os.path.join(SLAM_DIR, seq)
    gt = read_tum(os.path.join(folder, "stamp_gt_traj.txt"))
    xyz = gt.positions_xyz
    ax.plot(xyz[:, 0], xyz[:, 1], "--", color="gray", lw=1.2, label="ground truth")
    for name, label, color in (("stamp_est_traj_front_end.txt", "DRG odometry", FRONT_END_COLOR),
                               ("stamp_est_traj_no_loop_closure.txt", "+ back end", NO_LOOP_COLOR),
                               ("stamp_est_traj_loop_closure.txt", "+ back end + loop closure", DRG_COLOR)):
        est, rmse = aligned_with_ate(slam_ground_truth(folder, name, gt), read_tum(os.path.join(folder, name)))
        e = est.positions_xyz
        ax.plot(e[:, 0], e[:, 1], "-", color=color, lw=1.1, label=f"{label}: {rmse:.1f} m")
    length = np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()
    ax.set_title(f"{seq} ({length_text(length)})", fontsize=10)
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=6.5, loc="best", title="ATE (evo_ape -va)", title_fontsize=6.5)
    ax.tick_params(labelsize=7)
    ax.set_xlabel("x (m)", fontsize=8)


def overview_real():
    fig, axes = plt.subplots(1, 4, figsize=(14, 4.2))
    for ax, seq in zip(axes, ("nyl", "loop1", "loop2", "loop3")):
        panel_slam(ax, seq)
    axes[0].set_ylabel("y (m)", fontsize=8)
    fig.suptitle("Real trajectories, never restarted at the ground truth: DRG odometry and 4DRadarSLAM on it "
                 "(keyframes)", fontsize=10)
    fig.tight_layout()
    return fig


def overview_paper_vs_real():
    fig, axes = plt.subplots(2, 2, figsize=(10, 9))
    for col, seq in enumerate(("loop1", "loop2")):
        folder = os.path.join(ODOMETRY_DIR, seq)
        gt = read_tum(os.path.join(folder, "stamp_gt_traj.txt"))
        g = gt.positions_xyz
        whole = read_tum(os.path.join(folder, "stamp_est_traj.txt"))
        seg = read_tum(os.path.join(folder, "stamp_est_traj_segmented.txt"))
        starts = segment_starts(seq, seg)
        _, rmse_seg = aligned_with_ate(gt, seg)
        est, rmse = aligned_with_ate(gt, whole)

        ax = axes[0, col]
        ax.plot(g[:, 0], g[:, 1], "--", color="gray", lw=1.2, label="ground truth")
        p = seg.positions_xyz
        for k, (a, b) in enumerate(zip(starts[:-1], starts[1:])):
            ax.plot(p[a:b, 0], p[a:b, 1], "-", color=DRG_COLOR, lw=1.1,
                    label=f"DRG, {len(starts) - 1} segments (APE {rmse_seg:.1f} m)" if k == 0 else None)
            ax.plot(p[a, 0], p[a, 1], "o", color="black", ms=3,
                    label=f"restart at the ground truth ({len(starts) - 2}x)" if k == 1 else None)
        ax.set_title(f"{seq}: as in Fig. 4 of the paper (restarted at the ground truth)", fontsize=9)

        ax = axes[1, col]
        ax.plot(g[:, 0], g[:, 1], "--", color="gray", lw=1.2, label="ground truth")
        e = est.positions_xyz
        ax.plot(e[:, 0], e[:, 1], "-", color=DRG_COLOR, lw=1.1, label=f"DRG odometry (ATE {rmse:.1f} m)")
        ax.set_title(f"{seq}: real run, never restarted (evo_ape -va)", fontsize=9)
        for ax in axes[:, col]:
            mark_dropouts(ax, gt)
            ax.set_aspect("equal", adjustable="datalim")
            ax.legend(fontsize=7, loc="center" if seq == "loop1" else "upper right")
            ax.tick_params(labelsize=7)
    fig.tight_layout()
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--segmented", action="store_true",
                    help="also plot stamp_est_traj_segmented.txt as trajectory_segmented.pdf (Fig. 4 of the paper)")
    ap.add_argument("--overview", action="store_true", help="also write the two README figures in docs/figures")
    args = ap.parse_args()
    if args.overview:
        for figure, name in ((overview_real, "trajectories_real.png"),
                             (overview_paper_vs_real, "loop1_loop2_paper_vs_real.png")):
            path = os.path.join(REPO, "docs", "figures", name)
            fig = figure()
            fig.savefig(path, dpi=130, bbox_inches="tight")
            plt.close(fig)
            print(f"wrote {os.path.relpath(path, REPO)}")
    jobs = [(ODOMETRY_DIR, ODOMETRY_SEQUENCES, odometry_figure, "trajectory.pdf"),
            (SLAM_DIR, SLAM_SEQUENCES, slam_figure, "trajectory.pdf")]
    if args.segmented:
        jobs.append((ODOMETRY_DIR, ODOMETRY_SEQUENCES[:6], segmented_figure, "trajectory_segmented.pdf"))
    for folder, sequences, figure, name in jobs:
        for seq in sequences:
            path = os.path.join(folder, seq, name)
            fig = figure(seq)
            fig.savefig(path, bbox_inches="tight", metadata={"CreationDate": None})  # byte-stable rebuilds
            plt.close(fig)
            print(f"wrote {os.path.relpath(path, REPO)}")


if __name__ == "__main__":
    main()
