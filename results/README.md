# Released estimates

`relative_poses/<sequence>.txt` holds DRG's estimate for **every pair** behind Table 1 of the
paper (seed 42, 51,940 pairs), exactly as produced for the paper. One line per pair:

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
