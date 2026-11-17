# Tutorial data

`example.ipynb` walks through the six stages of DRG on the data in this folder and checks the
result against the paper. It needs no ROS and no dataset download.

| File | Content |
|---|---|
| `data/loop3_set4/<frame>/cloud.pcd` | 31 consecutive radar scans (frames 5692–5722) of the NTU4DRadLM sequence *loop3*: x, y, z, RCS (`intensity`) and Doppler (`curvature`); `normal_x` holds 4DRadarSLAM's DBSCAN cluster rank and `normal_y`, `normal_z` are zero. DRG reads only the first five |
| `data/loop3_set4/<frame>/data` | the radar timestamp of the scan (`stamp <sec> <nsec>`) |
| `data/loop3_gt_odom_excerpt.txt` | the ground truth (TUM format, 10 Hz) from 1 s before the first to 1 s after the last scan |
| `datasets.yaml` | lets `scripts/reproduce_windowed_sets.py` run on this window |

The clouds are copied unchanged from the per-scan dump the paper registered: 4DRadarSLAM
preprocessing of the `/radar_enhanced_pcl` scans, rotated into the body axes (x forward, y left,
z up, origin at the radar). The window is the paper's windowed set *loop3 set 4*. The car drives at
about 7.6 m/s, and DRG's mean errors on it are 0.0440 m and 0.133°.

```bash
# the notebook
pip3 install -r ../requirements.txt notebook
jupyter notebook example.ipynb

# or, from the repository root, the same 30 pairs against the paper's estimates
python3 scripts/reproduce_windowed_sets.py --datasets example/datasets.yaml --sets loop3_set4_5692
python3 scripts/register_dump.py --dump example/data/loop3_set4 --stats config/norm_stats/loop3.yaml \
    --gt example/data/loop3_gt_odom_excerpt.txt --out /tmp/drg_example
```

The data are an excerpt of [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM) (Zhang et al.,
2023). Please cite the dataset if you use them.
