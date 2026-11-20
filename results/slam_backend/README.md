# DRG in a full SLAM system (supplementary, not in the paper)

The paper evaluates DRG as scan-to-scan odometry. Here the same odometry is the front end of the
complete [4DRadarSLAM](https://github.com/zhuge2333/4DRadarSLAM) system (its
[Go-RIO](https://github.com/wooseongY/Go-RIO) version), with keyframes, a pose graph and loop
closure. These runs are not part of the paper.

## Setup

* **System.** The workspace's original launch files (`verify_dump_full_<seq>.launch`). The only
  change is that the `ScanMatchingOdometryNodelet` node block (Fast-APDGICP) is removed and DRG
  publishes the odometry instead: one `nav_msgs/Odometry` on `/odom` for every `/filtered_points`
  scan.
* **DRG odometry.** The paper's per-pair estimates (`../relative_poses/`), chained over every scan
  from the identity. Steps without a stored estimate, i.e. pairs left out of the paper's evaluation
  for lack of ground truth, are bridged at constant velocity: loop1 46 steps, loop2 19, loop3 10,
  garden 14, cp 2, nyl 0. On garden the run uses DRG on every scan, not the paper's 1 s keyframes.
  On cp, 2 of the 5,261 scans the back end receives (at 270.7 s and 276.2 s) are not in the scan
  set the paper evaluated, so they have no DRG pose and the back end skips them.
* **Back end, as shipped.** A keyframe every 0.5 m or 10°; a g2o pose graph with odometry edges and
  IMU / radar ego-velocity preintegration edges; Scan Context loop detection with loop verification
  (`enable_loop_closure:=true`, loop yaw gate `max_yaw_difference` = 20°). GPS and barometer are
  off, and no node reads the ground truth.

## Files

| file | content |
|---|---|
| `<seq>/stamp_est_traj.txt` | keyframe poses after pose-graph optimisation with loop closure: the SLAM output |
| `<seq>/stamp_est_traj_front_end.txt` | the same keyframes before the pose graph: DRG's odometry, chained over the whole run |
| `<seq>/stamp_gt_traj.txt` | NTU4DRadLM ground truth interpolated at the keyframe stamps (`scripts/interpolate_gt.py`) |
| `<seq>/trajectory.pdf` | top view of the three, both estimates SE(3)-aligned to the ground truth (`scripts/plot_trajectories.py`) |

The poses are exactly as the system wrote them, in its own odometry frame, so `evo_ape` must align
them (`-va`).

## Results

```bash
cd results/slam_backend
evo_ape tum loop3/stamp_gt_traj.txt loop3/stamp_est_traj.txt -va --plot --plot_mode xy
evo_ape tum loop3/stamp_gt_traj.txt loop3/stamp_est_traj_front_end.txt -va
```

ATE RMSE of `evo_ape -va` (SE(3) Umeyama alignment, evo 1.37.1) over the keyframes:

| sequence | ground-truth path | keyframes | loop closures | DRG front end [m] | DRG + back end [m] |
|---|---|---|---|---|---|
| cp | 229 m | 435 | 1 | 1.51 | **0.54** |
| garden | 324 m | 615 | 1 | 3.13 | 6.31 |
| nyl | 995 m | 1,777 | 0 | 9.40 | 9.35 |
| loop1 | 6,943 m | 8,025 | 0 | 550.40 | 548.84 |
| loop2 | 4,792 m | 6,162 | 0 | 120.38 | 112.11 |
| loop3 | 4,232 m | 6,118 | 52 | 59.20 | **9.46** |

Both columns come from the same run and the same keyframes, so they compare directly.

* **loop3.** 52 loop closures bring the ATE from 59.2 m to 9.5 m. This gain comes from the back end
  (loop closure, pose graph), not from DRG.
* **cp.** One loop closes when the car returns to its start: the two keyframes are 0.8 m apart in
  the ground truth. It brings the ATE from 1.51 m to 0.54 m. The same run without loop closure
  gives 1.59 m, so the gain comes from the loop, not from the pose graph.
* **loop1** never returns to a place it has passed, so no loop can close. Most of its error comes
  from one radar dropout in the raw rosbag (marked in black in `loop1/trajectory.pdf`): two
  consecutive scans are 17.4 s apart while the car drives 90 m and turns 43°. DRG, like
  4DRadarSLAM's own front end, estimates about 0.9 m and 0° for that pair, and the 43° heading error
  stays in every later pose. As a diagnostic, chaining the odometry over every scan with only that
  pair replaced by its ground-truth motion lowers the ATE from 553 m to 194 m. The back end does not
  bridge the gap either: its IMU preintegration edges are weighted with information 1/σ (0.005 in
  yaw), against 1.0 for an odometry edge, so the graph follows the odometry. Three repeated loop1
  runs agree to 0.04 m, so this is the system's result, not a run-to-run effect.
* **loop2.** With DRG's heading drift, the true revisits fail the shipped 20° yaw gate of the loop
  detector, so no loop closes.
* **garden.** The back end makes the result worse (3.1 m to 6.3 m), with one loop closure.

The front-end column is DRG's odometry over the whole run. It is the same trajectory as
`../trajectories/<seq>/stamp_est_traj.txt` (within 7 mm at every keyframe; not on garden, which ran
DRG on every scan), whose ATE over every scan is 552.8 m on loop1 and 60.7 m on loop3. The paper's
Fig. 4 instead restarts the chain at the ground truth after each missing pair, which gives much lower
values (loop1: 20.7 m, loop3: 15.0 m); on loop1 the next restart comes 30 s after the dropout. See
[`../README.md`](../README.md#odometry-trajectories).

The ground-truth files are derived from the [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM)
ground truth. Please cite NTU4DRadLM when you use them.
