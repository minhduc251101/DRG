# Released results

Everything here comes from the estimates of the paper (seed 42) for the NTU4DRadLM sequences:

* [`relative_poses/`](#per-pair-estimates): DRG's estimate for every scan pair behind Tables 1 and 3
* [`trajectories/`](#odometry-trajectories): the same estimates chained over the whole run into
  odometry trajectories, with the ground truth at the same stamps (TUM format, for
  [evo](https://github.com/MichaelGrupp/evo)), the ATE and a plot of each sequence. This is also the
  odometry the SLAM back end receives
* [`slam_backend/`](slam_backend/README.md): **not in the paper.** DRG as the front end of the full
  4DRadarSLAM system (keyframes, pose graph, loop closure), with the absolute trajectory error (ATE)
  of each sequence

## Per-pair estimates

`relative_poses/<sequence>.txt` holds DRG's estimate for **every pair** behind Table 1 of the
paper (51,940 pairs), exactly as produced for the paper. One line per pair:

```
stamp_target stamp_source  r11 r12 r13 tx  r21 r22 r23 ty  r31 r32 r33 tz
```

**T** maps points of the source scan (the later one) into the frame of the target scan. The
frame is the radar-centred body frame (radar origin, Livox axes).

| sequence | pairs | mean TE [m] | mean RE [°] | *t*_rel [%] | *r*_rel [°/m] | `--max-gt-gap` |
|---|---|---|---|---|---|---|
| cp | 5,256 | 0.0135 | 0.189 | 1.55 | 0.0465 | 0.1 |
| garden (keyframes) | 272 | 0.0554 | 0.555 | 6.56 | 0.0761 | 0.1 |
| nyl | 11,243 | 0.0229 | 0.223 | 5.83 | 0.0225 | 0.15 |
| loop1 | 15,992 | 0.0736 | 0.101 | 9.03 | 0.0298 | 0.1 |
| loop2 | 11,568 | 0.0687 | 0.142 | 7.30 | 0.0242 | 0.1 |
| loop3 | 7,609 | 0.0778 | 0.114 | 5.94 | 0.0240 | 0.1 |

Recompute any row with the dataset's `gt_odom.txt`:

```bash
python3 scripts/evaluate_pairs.py      --est results/relative_poses/loop2.txt --gt <NTU4DRadLM>/loop2/gt_odom.txt
python3 scripts/evaluate_trajectory.py --est results/relative_poses/loop2.txt --gt <NTU4DRadLM>/loop2/gt_odom.txt
```

Pairs are missing where a frame has no ground truth within the gap limit (0–0.3 % per
sequence). For *t*_rel and *r*_rel, `evaluate_trajectory.py` cuts the chain at each missing pair and
scores each piece on its own, which is why loop1 has 22 chains.

## Odometry trajectories

`trajectories/<sequence>/` holds:

| file | content |
|---|---|
| `stamp_est_traj.txt` | DRG: the per-pair estimates of `relative_poses/` chained over the whole run, P<sub>k+1</sub> = P<sub>k</sub> T<sub>k</sub>, from the ground-truth pose of the first scan, with no keyframing, smoothing, loop closure or restart |
| `stamp_gt_traj.txt` | NTU4DRadLM ground truth interpolated at the same radar stamps (linear in position, SLERP in rotation, as in the paper) |
| `trajectory.pdf` | top view of both, the estimate SE(3)-aligned to the ground truth as by `evo_ape -va`, with its ATE in the legend |

The two TUM files (`timestamp tx ty tz qx qy qz qw`; lines starting with `#` are comments) have the
same stamps, so evo pairs every pose exactly. Scans without ground truth are left out, and the step
across each such gap is bridged at constant velocity (the last estimated translation per second, no
rotation), the same as in the odometry DRG feeds to the SLAM back end.

| sequence | poses | bridged gaps | length [m] | ATE [m] |
|---|---|---|---|---|
| cp | 5,259 | 2 | 240 | 1.55 |
| garden (keyframes) | 273 | 0 | 142 | 1.02 |
| nyl | 11,244 | 0 | 1,009 | 9.66 |
| loop1 | 16,014 | 21 | 6,944 | 552.85 |
| loop2 | 11,578 | 9 | 4,793 | 122.26 |
| loop3 | 7,615 | 5 | 4,233 | 60.73 |

ATE is the RMSE of `evo_ape -va` (SE(3) Umeyama alignment) over every scan:

```bash
cd results/trajectories
evo_ape tum loop3/stamp_gt_traj.txt loop3/stamp_est_traj.txt -va --plot --plot_mode xy
```

The paper reports no ATE. It scores the chained odometry by *t*_rel and *r*_rel (Table 3), which
`scripts/evaluate_trajectory.py` computes from `relative_poses/`, not from these files.

**Same odometry as the SLAM front end.** These trajectories are the odometry that DRG feeds to the
SLAM back end in [`slam_backend/`](slam_backend/README.md). At the back end's keyframes, the two agree
to within 7 mm in position over the whole run (cp, nyl, loop1, loop2 and loop3; after a common rigid
offset). The ATE here therefore matches the "DRG front end" column there; it differs slightly only
because it is taken over every scan, not over the keyframes. Garden is the exception: here it uses
the paper's 1 s keyframes, while the back end ran DRG on every scan.

**loop1 and the radar dropout.** In the raw rosbag `loop1_2022-06-03_1.bag`, `/radar_enhanced_pcl`
has no message for 17.4 s (stamps 1654229729.355 to 1654229746.745). The IMU keeps publishing
throughout. Meanwhile the car drives 90 m and turns 43°, so the two scans either side of the gap
are a single pair, and no scan-to-scan method can recover that motion from them: DRG and
4DRadarSLAM's own front end both estimate about 0.9 m and 0°. The 43° heading error of that one pair
stays in every later pose, which is most of loop1's 553 m ATE. Replacing only that pair by its
ground-truth motion gives 194 m. The plots mark the dropout in black. The other long gaps between
scans (cp, loop2, nyl, garden) are stops: the ground truth moves at most 0.1 m over them.

For per-pair errors use `scripts/evaluate_pairs.py`: `evo_rpe` on these files would also score the
bridged steps.

Rebuild the files from `relative_poses/` and the dataset's `gt_odom.txt` (paths and gap limits in
`config/datasets.yaml`, see `config/datasets_example.yaml`), then the plots of both folders:

```bash
python3 scripts/export_trajectories.py --datasets config/datasets.yaml
python3 scripts/plot_trajectories.py
```

**The version of the paper's Fig. 4.** Fig. 4 restarts the chain at the ground truth after each
missing pair, so the drift inside a segment is front-end drift alone. On loop1 the next restart comes
30 s after the dropout and keeps its heading error local: `evo_ape` gives 20.7 m on that version,
against 552.8 m over the whole run. `--segmented` rebuilds it as `stamp_est_traj_segmented.txt` and
`trajectory_segmented.pdf`:

```bash
python3 scripts/export_trajectories.py --datasets config/datasets.yaml --segmented
python3 scripts/plot_trajectories.py --segmented
```

The ground-truth files are derived from the [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM)
ground truth. Please cite NTU4DRadLM when you use them.
