# Released results

Everything here comes from the estimates of the paper (seed 42) for the six NTU4DRadLM sequences:

* [`relative_poses/`](#per-pair-estimates): DRG's estimate for every scan pair behind Tables 1 and 3
* [`trajectories/`](#trajectories-for-evo): the same estimates chained into trajectories, with the
  ground truth at the same stamps, in TUM format for [evo](https://github.com/MichaelGrupp/evo)

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
sequence). A trajectory chain is cut at each missing pair, which is why loop1 has 22 chains.

## Trajectories for evo

`trajectories/<sequence>/` holds two TUM files (`timestamp tx ty tz qx qy qz qw`; lines starting
with `#` are comments) with the same stamps, so evo pairs every pose exactly:

| file | content |
|---|---|
| `stamp_est_traj.txt` | DRG: the per-pair estimates of `relative_poses/` chained, P<sub>k+1</sub> = P<sub>k</sub> T<sub>k</sub>, with no keyframing, smoothing or loop closure |
| `stamp_gt_traj.txt` | NTU4DRadLM ground truth interpolated at the same radar stamps (linear in position, SLERP in rotation, as in the paper) |

These are the trajectories behind *t*_rel and *r*_rel in Table 3. Each chain starts at the
ground-truth pose of its first scan, so every pose is in the ground-truth world frame and the
error measures the drift within a chain. A plot draws each new chain as a straight jump back
to the ground truth.

```bash
cd results/trajectories
evo_ape tum loop3/stamp_gt_traj.txt loop3/stamp_est_traj.txt -va --plot --plot_mode xy
evo_traj tum loop3/stamp_est_traj.txt --ref loop3/stamp_gt_traj.txt --plot --plot_mode xy

# all six sequences
for s in cp garden nyl loop1 loop2 loop3; do evo_ape tum $s/stamp_gt_traj.txt $s/stamp_est_traj.txt -va; done
```

`evo_ape -va` (SE(3) Umeyama alignment, evo 1.37.1) on these files gives:

| sequence | poses | chains | length [m] | APE RMSE [m] | APE mean [m] | APE max [m] |
|---|---|---|---|---|---|---|
| cp | 5,259 | 3 | 240 | 1.240 | 1.130 | 2.186 |
| garden (keyframes) | 273 | 1 | 142 | 1.016 | 0.910 | 2.428 |
| nyl | 11,244 | 1 | 1,009 | 9.656 | 8.119 | 24.300 |
| loop1 | 16,014 | 22 | 6,935 | 20.655 | 15.995 | 102.708 |
| loop2 | 11,578 | 10 | 4,790 | 52.942 | 40.555 | 186.624 |
| loop3 | 7,615 | 6 | 4,230 | 14.997 | 13.493 | 42.036 |

The paper does not report APE. DRG is scan-to-scan registration without a back end, so the
absolute error grows with the length of a chain; the paper's trajectory metric is *t*_rel /
*r*_rel (Table 3). For per-pair errors use `scripts/evaluate_pairs.py`: `evo_rpe` on these files
would also compare the last pose of each chain with the first pose of the next one.

Rebuild the files from `relative_poses/` and the dataset's `gt_odom.txt` (paths and gap limits in
`config/datasets.yaml`, see `config/datasets_example.yaml`):

```bash
python3 scripts/export_trajectories.py --datasets config/datasets.yaml
```

The ground-truth files are derived from the [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM)
ground truth. Please cite NTU4DRadLM when you use them.
