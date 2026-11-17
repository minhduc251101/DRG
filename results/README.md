# Released results

Everything here comes from the estimates of the paper (seed 42) for the NTU4DRadLM sequences:

* [`relative_poses/`](#per-pair-estimates): DRG's estimate for every scan pair behind Tables 1 and 3
* [`trajectories/`](#odometry-trajectories): the same estimates chained into odometry trajectories, as
  in Fig. 4 of the paper, with the ground truth at the same stamps (TUM format, for
  [evo](https://github.com/MichaelGrupp/evo)) and a plot of each sequence
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
sequence). A trajectory is cut at each missing pair, which is why loop1 has 22 segments.

## Odometry trajectories

`trajectories/<sequence>/` holds:

| file | content |
|---|---|
| `stamp_est_traj.txt` | DRG: the per-pair estimates of `relative_poses/` chained, P<sub>k+1</sub> = P<sub>k</sub> T<sub>k</sub>, with no keyframing, smoothing or loop closure |
| `stamp_gt_traj.txt` | NTU4DRadLM ground truth interpolated at the same radar stamps (linear in position, SLERP in rotation, as in the paper) |
| `trajectory.pdf` | top view of both, drawn with evo's plotting module |

The two TUM files (`timestamp tx ty tz qx qy qz qw`; lines starting with `#` are comments) have
the same stamps, so evo pairs every pose exactly. As in Fig. 4 of the paper, each segment starts
at the ground-truth pose of its first scan, so the drift inside a segment is front-end drift
alone. These are the trajectories behind *t*_rel and *r*_rel in Table 3.

| sequence | poses | segments | length [m] |
|---|---|---|---|
| cp | 5,259 | 3 | 240 |
| garden (keyframes) | 273 | 1 | 142 |
| nyl | 11,244 | 1 | 1,009 |
| loop1 | 16,014 | 22 | 6,935 |
| loop2 | 11,578 | 10 | 4,790 |
| loop3 | 7,615 | 6 | 4,230 |

```bash
cd results/trajectories
evo_traj tum loop3/stamp_est_traj.txt --ref loop3/stamp_gt_traj.txt --plot --plot_mode xy
```

**Absolute trajectory error.** The paper reports no ATE; it scores the chained odometry by
*t*_rel and *r*_rel (Table 3). Because every segment restarts at the ground truth, `evo_ape` on
these files measures the drift within segments, not the ATE of a whole run: loop3 gives 15.0 m
here, against 59.2 m for DRG's odometry over the whole run. Whole-run ATEs, of DRG alone and with a
SLAM back end, are in [`slam_backend/`](slam_backend/README.md). For per-pair errors use
`scripts/evaluate_pairs.py`: `evo_rpe` on these files would also compare the last pose of a
segment with the first pose of the next.

Rebuild the files from `relative_poses/` and the dataset's `gt_odom.txt` (paths and gap limits in
`config/datasets.yaml`, see `config/datasets_example.yaml`), then the plots of both folders:

```bash
python3 scripts/export_trajectories.py --datasets config/datasets.yaml
python3 scripts/plot_trajectories.py
```

The ground-truth files are derived from the [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM)
ground truth. Please cite NTU4DRadLM when you use them.
