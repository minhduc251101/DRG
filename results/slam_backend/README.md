# DRG in a full SLAM system, with and without loop closure (not in the paper)

The paper evaluates DRG as scan-to-scan odometry. Here the same odometry is the front end of the
complete [4DRadarSLAM](https://github.com/zhuge2333/4DRadarSLAM) system (its
[Go-RIO](https://github.com/wooseongY/Go-RIO) version), run from the NTU4DRadLM rosbags with loop closure off and on. Because the back end is not
deterministic, the runs with loop closure were repeated on loop2 (3 runs) and loop3
(7 runs). These runs are not part of the paper.

## Setup

* **System.** The workspace's original launch files (`verify_dump_full_<seq>.launch`). The only
  change is that the `ScanMatchingOdometryNodelet` node block (Fast-APDGICP) is removed and DRG
  provides the odometry instead: one `nav_msgs/Odometry` on `/odom` for every `/filtered_points` scan.
* **DRG odometry.** DRG's estimates for every consecutive pair of `/filtered_points` scans, from the
  rerun of this repository's code (`../relative_poses/`), chained from the identity. Every step is a DRG
  estimate: nothing is bridged or extrapolated. DRG's Python implementation is slower than real
  time (see the timing in the paper), so the estimates were computed offline and a small node publishes, for each `/filtered_points`
  message, the pose of the scan with the same stamp. The back end therefore receives exactly the
  odometry the DRG ROS node computes on these scans (the node matches the library to about 1e-11 m).
  On garden DRG runs on every scan (`../relative_poses/garden_every_scan.txt`), not on the paper's 1 s
  keyframes. On cp, the scan set the paper evaluated has no point cloud for 2 of the 5,261 scans the back
  end receives (dump frames 3247 and 3312), so they get no DRG pose and the back end skips them; the
  odometry crosses them with DRG's estimates of the pairs that span them.
* **Back end, as shipped.** A keyframe every 0.5 m or 10°; a g2o pose graph with odometry edges and
  IMU / radar ego-velocity preintegration edges; Scan Context loop detection with loop verification and
  the shipped loop yaw gate `max_yaw_difference` = 20°. Runs without and with loop
  closure differ only in `enable_loop_closure` (`false` / `true`). GPS and barometer are off, and the runner aborts if any node
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
| `<seq>/repeats/run<k>/` (loop2, loop3) | the other runs with loop closure on the same input (`<seq>/` holds the best run): keyframe poses, ground truth and `evo_ape -va` results |

The poses are exactly as the system wrote them, in its own odometry frame, so `evo_ape` must align
them (`-va`). `bash scripts/evo_ape_all.sh` reruns every evo command of `results/`.

## Results

ATE (RMSE of `evo_ape -va`, SE(3) Umeyama alignment, evo 1.37.1) over the keyframes of each run:

| sequence | ground-truth path (at the keyframes) | keyframes | DRG odometry [m] | back end, no loop closure [m] | back end, loop closure, best run [m] | loop closures | loop-closure runs (median [m]) |
|---|---|---|---|---|---|---|---|
| cp | 229 m | 435 | 1.57 | 1.66 | 0.51 | 1 | 1 (0.51) |
| garden | 324 m | 615 / 615 | 3.15 | 7.09 | 6.30 | 1 | 1 (6.30) |
| nyl | 995 m | 1,777 | 9.40 | 9.35 | 7.50 | 0 | 1 (7.50) |
| loop1 | 6,943 m | 8,025 | 536.44 | 535.14 | 535.15 | 0 | 1 (535.15) |
| loop2 | 4,792 m | 6,163 | 120.01 | 111.72 | 14.09 | 47 | 3 (57.26) |
| loop3 | 4,232 m | 6,118 | 56.85 | 55.41 | 11.85 | 55 | 7 (20.52) |

```bash
cd results/slam_backend/loop3
evo_ape tum stamp_gt_traj.txt stamp_est_traj_loop_closure.txt -va --save_results evo_ape_loop_closure.zip --save_plot evo_ape_loop_closure.pdf --plot_mode xy
```

The two runs of a sequence (loop closure off / on) selected the same keyframes, except on garden (a few keyframes differ; each estimate is scored at its own), and DRG's odometry at them is the same.

* **With loop closure the result varies from run to run.** 4DRadarSLAM's back end is not
  deterministic (its pose-graph optimisation runs on a wall-clock timer), and on loop2 and loop3 the loops it
  accepts, and with them the ATE, change between runs of the same input. loop2 was run 3 times
  and loop3 7 times with loop closure; the table and the files in `<seq>/` are the best run
  (lowest ATE), every other run is in `<seq>/repeats/`. A rerun lands somewhere in this range, not necessarily
  on the best run: compare with the median.
  * **loop2**, runs 1 to 3: 14.09 m (47 loop closures), 57.26 m (34), 101.11 m (1); best: run 1,
    median 57.26 m. Without loop closure: 111.72 m. The result hinges
    on the first loop candidate, a true revisit (keyframes 5032 and 66,
    1.4 m apart in the ground truth). The loop verification accepted it in
    runs 1 and 2 (χ² after verification 0.000 and 0.000;
    threshold 5000), and most later revisits then passed as well
    (47 of 47 and 34 of 36 candidates). In
    run 3 it rejected that revisit (χ² 18,702) and accepted one false
    loop instead (keyframes 5500 and 18, 304 m apart in the ground truth).
    With the odometry used before this rerun, in which the pairs without ground truth were bridged at
    constant velocity instead of estimated by DRG, the first revisit was rejected in both runs
    (χ² 19,219 in the first), so no loop closed.
  * **loop3**, runs 1 to 7: 11.98 m (55 loop closures), 21.23 m (54), 21.63 m (54), 20.52 m (53), 11.88 m (55), 11.85 m (55), 21.45 m (54); best: run 6,
    median 20.52 m. Without loop closure: 55.41 m. Every accepted
    loop of the 7 runs is a true revisit (at most 4.3 m apart in the ground truth).
    The 3 runs that closed 55 loops end at 11.85 to 11.98 m, the 4 that closed 53 or 54 at 20.52 to 21.63 m. In those 4 runs the loop verification rejected 1 or 2 true revisits, always including keyframes 5238 and 146 (3.4 m apart in the ground truth); in the other 3 it rejected none. The odometry used before this rerun gave
    9.46 m (52 loop closures) on
    17 Sep and, run again today, 15.76 m
    (52).
* **cp.** 1 loop closure when the handcart returns to its start (keyframes
  418 and 1, 1.5 m apart in the ground truth):
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
* **garden.** 1 loop closure, at the return to the start (keyframes 597
  and 1, 0.1 m apart in the ground truth). The back end makes garden worse than the odometry
  alone: 3.15 m for the odometry, 7.09 m without and
  6.30 m with loop closure.

The odometry column differs slightly from the whole-run ATE of `../trajectories/` because it is taken at
the keyframes only, not at every scan. The back end, the loop detector and the pose graph are as shipped
in 4DRadarSLAM.

## Reproduce

**(a) Re-score the released runs (seconds).** Every number above is `evo_ape -va` on the files in this folder:

```bash
bash scripts/evo_ape_all.sh slam_backend
```

**(b) Run the SLAM system again.** Needs ROS Noetic, the [Go-RIO](https://github.com/wooseongY/Go-RIO) version
of 4DRadarSLAM and the NTU4DRadLM rosbags. Because the back end is not deterministic, loop2 and loop3 with loop
closure will land somewhere in the range of the runs listed above, not necessarily on the best one.

```bash
# 1. DRG odometry for every scan the back end receives (from the released estimates; garden: garden_every_scan.txt)
python3 scripts/slam/build_odom_table.py --relative-poses results/relative_poses/loop3.txt --out drg_odom_loop3.txt

# 2. The sequence's 4DRadarSLAM launch file without its own front end: copy it, delete the
#    scan_matching_odometry_nodelet node block, and set the parameters of config/4dradarslam/backend_params.yaml
#    (all six sequences used the same values; never set gt_file_location). Then check the copy:
python3 scripts/slam/check_launch.py <4DRadarSLAM>/launch/drg_loop3.launch

# 3. Run, in this order (as in our runs)
roscore &
until rostopic list >/dev/null 2>&1; do sleep 1; done          # wait for the master
#    loop-candidate ellipse: the library defaults, set explicitly. No launch file sets these two
#    parameters, so roslaunch keeps them.
rosparam set /radar_graph_slam_nodelet/odometry_drift_xy 0.05
rosparam set /radar_graph_slam_nodelet/odometry_drift_scale_xy 2.0
#    the DRG odometry publisher must be up before the bag plays, or the first scans get no /odom
python3 scripts/slam/odom_table_publisher.py _table:=$PWD/drg_odom_loop3.txt &
until rostopic info /odom >/dev/null 2>&1; do sleep 1; done    # wait until /odom is advertised
roslaunch <4DRadarSLAM package> drg_loop3.launch enable_loop_closure:=true enable_loop_verification:=true &
#    after start-up, check the loop gate the back end holds: 0.05, 2.0 and 20 (degrees)
rosparam get /radar_graph_slam_nodelet/odometry_drift_xy
rosparam get /radar_graph_slam_nodelet/odometry_drift_scale_xy
rosparam get /radar_graph_slam_nodelet/max_yaw_difference
#    when the bag has finished and the back end's output has stopped for 4 minutes (our runs waited 240 s):
rosservice call /radar_graph_slam/dump "destination: '$PWD/dump'"

# 4. Score
python3 scripts/slam/dump_to_tum.py --dump dump --out run --name loop_closure
python3 scripts/interpolate_gt.py --gt <NTU4DRadLM>/loop3/gt_odom.txt --est run/stamp_est_traj_loop_closure.txt \
    --out run/stamp_gt_traj.txt
cd run && evo_ape tum stamp_gt_traj.txt stamp_est_traj_loop_closure.txt -va --save_results evo_ape_loop_closure.zip \
    --save_plot evo_ape_loop_closure.pdf --plot_mode xy
```

For the run without loop closure use `enable_loop_closure:=false` and `--name no_loop_closure`.

The ground-truth files are derived from the [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM)
ground truth. Please cite NTU4DRadLM when you use them.
