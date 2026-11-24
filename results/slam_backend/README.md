# DRG in a full SLAM system, with and without loop closure (not in the paper)

The paper evaluates DRG as scan-to-scan odometry. Here the same odometry is the front end of the
complete [4DRadarSLAM](https://github.com/zhuge2333/4DRadarSLAM) system (its
[Go-RIO](https://github.com/wooseongY/Go-RIO) version), run from the NTU4DRadLM rosbags twice per
sequence: once with loop closure off and once with it on. These runs are not part of the paper.

## Setup

* **System.** The workspace's original launch files (`verify_dump_full_<seq>.launch`). The only
  change is that the `ScanMatchingOdometryNodelet` node block (Fast-APDGICP) is removed and DRG
  provides the odometry instead: one `nav_msgs/Odometry` on `/odom` for every `/filtered_points` scan.
* **DRG odometry.** DRG's estimates for every consecutive pair of `/filtered_points` scans, from the
  rerun of this repository's code (`../relative_poses/`), chained from the identity. Every step is a DRG
  estimate: nothing is bridged or extrapolated. DRG (Python, about 1–5 s per pair) is slower than real
  time, so the estimates were computed offline and a small node publishes, for each `/filtered_points`
  message, the pose of the scan with the same stamp. The back end therefore receives exactly the
  odometry the DRG ROS node computes on these scans (the node matches the library to about 1e-11 m).
  On garden DRG runs on every scan (`../relative_poses/garden_every_scan.txt`), not on the paper's 1 s
  keyframes. On cp, the scan set the paper evaluated has no point cloud for 2 of the 5,261 scans the back
  end receives (dump frames 3247 and 3312), so they get no DRG pose and the back end skips them; the
  odometry crosses them with DRG's estimates of the pairs that span them.
* **Back end, as shipped.** A keyframe every 0.5 m or 10°; a g2o pose graph with odometry edges and
  IMU / radar ego-velocity preintegration edges; Scan Context loop detection with loop verification and
  the shipped loop yaw gate `max_yaw_difference` = 20°. The two runs differ only in
  `enable_loop_closure` (`false` / `true`). GPS and barometer are off, and the runner aborts if any node
  could read the ground truth.

## Files

| file | content |
|---|---|
| `<seq>/stamp_est_traj_front_end.txt` | DRG's odometry at the keyframes, before the pose graph |
| `<seq>/stamp_est_traj_no_loop_closure.txt` | keyframe poses after the pose graph, loop closure off |
| `<seq>/stamp_est_traj_loop_closure.txt` | keyframe poses after the pose graph, loop closure on: the SLAM output |
| `<seq>/stamp_gt_traj.txt` | NTU4DRadLM ground truth interpolated at the keyframe stamps (linear / SLERP) |
| `<seq>/evo_ape_<estimate>.{txt,zip,pdf}` | `evo_ape tum stamp_gt_traj.txt stamp_est_traj_<estimate>.txt -va --save_results evo_ape_<estimate>.zip --save_plot evo_ape_<estimate>.pdf --plot_mode xy`: printed output, saved results, plot |
| `<seq>/trajectory.pdf` | top view of the three, each SE(3)-aligned to the ground truth as by `evo_ape -va` (`scripts/plot_trajectories.py`) |
| `<seq>/repeats/run2/`, `run3/` (loop2, loop3) | two more runs with loop closure on the same input: keyframe poses, ground truth and `evo_ape -va` results |

The poses are exactly as the system wrote them, in its own odometry frame, so `evo_ape` must align
them (`-va`). `bash scripts/evo_ape_all.sh` reruns every evo command of `results/`.

## Results

ATE (RMSE of `evo_ape -va`, SE(3) Umeyama alignment, evo 1.37.1) over the keyframes of each run:

| sequence | ground-truth path | keyframes | DRG odometry [m] | back end, no loop closure [m] | back end, loop closure [m] | loop closures |
|---|---|---|---|---|---|---|
| cp | 229 m | 435 | 1.57 | 1.66 | 0.51 | 1 |
| garden | 324 m | 615 / 615 | 3.15 | 7.09 | 6.30 | 1 |
| nyl | 995 m | 1,777 | 9.40 | 9.35 | 7.50 | 0 |
| loop1 | 6,943 m | 8,025 | 536.44 | 535.14 | 535.15 | 0 |
| loop2 | 4,792 m | 6,163 | 120.01 | 111.72 | 14.09 | 47 |
| loop3 | 4,232 m | 6,118 | 56.85 | 55.41 | 11.98 | 55 |

```bash
cd results/slam_backend/loop3
evo_ape tum stamp_gt_traj.txt stamp_est_traj_loop_closure.txt -va --save_results evo_ape_loop_closure.zip --save_plot evo_ape_loop_closure.pdf --plot_mode xy
```

The two runs of a sequence (loop closure off / on) selected the same keyframes, except on garden (a few keyframes differ; each estimate is scored at its own), and DRG's odometry at them is the same.

* **With loop closure the result varies from run to run.** 4DRadarSLAM's back end is not
  deterministic (its pose-graph optimisation runs on a wall-clock timer), and on loop2 and loop3 the loops it
  accepts, and with them the ATE, change between runs of the same input. Every run is listed here and its
  files are in `<seq>/repeats/`:
  * **loop2.** The first candidate is a true revisit, keyframes 5032 and 66
    (1.4 m apart in the ground truth). In this run its χ² after the verification is
    0.000 (accepted, threshold 5000); 47 loop closures follow, all true
    revisits, and the ATE drops from 111.72 m to 14.09 m.
    Two more runs with the same odometry gave 57.26 m (34 loop
    closures) and 101.11 m (1): in the last one the same first candidate
    had χ² 18,702 and was rejected. With the odometry used before this
    rerun, in which the pairs without ground truth were bridged at constant velocity instead of estimated
    by DRG, the same candidate had χ² 19,219 and was rejected in both runs, so no loop
    closed.
  * **loop3.** 55 loop closures, all true revisits:
    55.41 m without and 11.98 m with loop closure.
    Two more runs gave 21.23 m (54 loop closures) and
    21.63 m (54). The odometry used before this rerun gave
    9.46 m (52 loop closures) and,
    run again today, 15.76 m (52).
* **cp.** 1 loop closure when the handcart returns to its start:
  1.66 m without, 0.51 m with loop closure.
* **loop1** never returns to a place it has passed (0 true revisits: ground-truth positions within 10 m
  and at least 50 m apart along the path), so no loop can close (0 loop closures).
  Most of its error comes from one radar dropout in the raw rosbag (marked in black in
  `loop1/trajectory.pdf`): two consecutive scans are 17.4 s apart while the car drives
  90 m and turns 43°. DRG, like any scan-to-scan method, estimates
  0.46 m and 0° for that pair, and the heading error stays in every later
  pose. The back end does not bridge the gap: its IMU preintegration edges are weighted with information
  1/σ (0.005 in yaw), against 1.0 for an odometry edge, so the graph follows the odometry.
* **nyl.** 0 loop closures in either run (12 true revisits, none within the yaw
  gate), so the difference between 9.35 m without and
  7.50 m with loop closure is the back end's run-to-run variation, not loop closure.
* **garden.** 1 loop closure. The back end makes garden worse than the odometry
  alone: 3.15 m for the odometry, 7.09 m without and
  6.30 m with loop closure.

The odometry column differs slightly from the whole-run ATE of `../trajectories/` because it is taken at
the keyframes only, not at every scan. The back end, the loop detector and the pose graph are as shipped
in 4DRadarSLAM.

The ground-truth files are derived from the [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM)
ground truth. Please cite NTU4DRadLM when you use them.
