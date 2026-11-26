<div align="center">
    <h1>DRG</h1>
    <h3>4D Radar Scan-to-Scan Registration using Doppler Translation Fusion</h3>
    <a href="#installation"><img src="https://img.shields.io/badge/Python-3.8%2B-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="#run-on-a-rosbag-ros-noetic"><img src="https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros" /></a>
    <a href="#installation"><img src="https://img.shields.io/badge/Linux-Ubuntu%2020.04-FCC624?logo=linux&logoColor=black" /></a>
    <a href="#citation"><img src="https://img.shields.io/badge/ACRA-2026-b33737" /></a>
    <a href="LICENSE"><img src="https://img.shields.io/badge/License-MIT-green" /></a>
  <br />
  <br />
  <p><strong><em>Training-free registration of consecutive 4D radar scans with Doppler velocity and radar cross section (RCS)</em></strong></p>
</div>

---

## 📄 Resources

- **Paper**: *4D Radar Scan-to-Scan Registration using Doppler Translation Fusion*, Van Minh Duc Nguyen,
  Tiancheng Li, Zhenzi Li, Alen Alempijevic, Shoudong Huang, Australasian Conference on Robotics and
  Automation (ACRA), 2026.
- **Tutorial**: 📂 [`example/example.ipynb`](example/example.ipynb), the six stages on real radar data. It needs no ROS and no dataset download.
- **Setup guide for coding agents**: 📂 [`AGENTS.md`](AGENTS.md), step by step from a fresh machine to the paper's Tables 1 and 3, with the expected output of every command. A coding agent can follow it as it is, and so can a person. It opens with the prompt that hands it to Claude Code, see [Reproduce with Claude Code](#-reproduce-with-claude-code).
- **Released results**: 📂 [`results/`](results/README.md), all from a rerun of this code: DRG's estimate for every consecutive scan pair of the six sequences (the paper's Tables 1 and 3 are recomputed from them), the whole-run trajectories with their `evo_ape -va` results and plots, and the same odometry in the full 4DRadarSLAM system with and without loop closure. Not in the paper: the same code on SNAIL-Radar ([`results/snail_radar/`](results/snail_radar/README.md)).

<p align="center">
  <img src="docs/figures/drg_loop3.gif" alt="DRG on NTU4DRadLM loop3" width="640"/>
</p>

**DRG** (Doppler–RCS Gated registration) estimates the rigid transform between two consecutive scans
of a 4D imaging radar. It needs no training. It uses the two radar-native cues that geometric
pipelines ignore: the **Doppler velocity** and the **radar cross section (RCS)** of every return.

## 🍎 How DRG works

<p align="center"><img src="docs/figures/pipeline.png" width="600"></p>

| Stage | What it does | Code |
|---|---|---|
| 1 Soft-REVE | Ego-velocity **v̂** from Doppler (3-point RANSAC + least squares). Every return gets a Doppler residual *e*ᵢ and a soft confidence *w*ᵢ | [`ego_velocity.py`](src/drg/ego_velocity.py) |
| 2 37-D descriptor | 33-D FPFH + 4 radar channels [RCS, *w*ᵢ, neighbour count, height] | [`descriptor.py`](src/drg/descriptor.py) |
| 3 Matching | Mutual nearest neighbours in the 37-D space | [`correspondence.py`](src/drg/correspondence.py) |
| 4 Doppler–RCS gate | Keep a match only if \|Δρ\| ≤ 1 dB and \|Δe\| ≤ 0.01 m/s | [`correspondence.py`](src/drg/correspondence.py) |
| 5 Geometry | Correspondence RANSAC + truncated-Cauchy IRLS → **R**, **t**_geo | [`solver.py`](src/drg/solver.py) |
| 6 Doppler fusion | **t** = ω **t**_dop + (1−ω) **t**_geo. The weight ω depends on Δt alone: 0.93 at 12 Hz, 0.08 at 1 s | [`fusion.py`](src/drg/fusion.py) |

<p align="center"><img src="docs/figures/cues_and_fusion.png" width="900"></p>

## 📊 Results

The DRG numbers in this section were recomputed on 24 Nov 2026 by **rerunning this repository's code**
on every consecutive scan pair of six [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM) sequences
(Oculii Eagle, 12 Hz) and scoring the output against the ground truth; none is copied from the paper.
The comparison with other methods is the paper's (they were not rerun). See
[Reproducing the paper result](#reproducing-the-paper-result).

### Per-pair accuracy and drift (Tables 1 and 3 of the paper)

The paper scores every consecutive pair whose two frames have ground truth (garden: its ~1 s
keyframes), 51,940 pairs in total. TE [m] and RE [°] are the translation and rotation errors of each
pair against the ground truth. *t*_rel and *r*_rel are the KITTI drift of the chained estimates over
100–800 m.

| DRG | cp | garden | nyl | loop1 | loop2 | loop3 |
|---|---|---|---|---|---|---|
| platform | handcart | handcart | handcart | car | car | car |
| pairs scored | 5,256 | 272 | 11,243 | 15,992 | 11,568 | 7,609 |
| mean TE [m] | 0.0135 | 0.0554 | 0.0229 | 0.0736 | 0.0687 | 0.0778 |
| mean RE [°] | 0.189 | 0.555 | 0.223 | 0.101 | 0.142 | 0.114 |
| median TE [m] | 0.0104 | 0.0459 | 0.0164 | 0.0487 | 0.0447 | 0.0532 |
| median RE [°] | 0.156 | 0.473 | 0.188 | 0.062 | 0.084 | 0.084 |
| *t*_rel [%] (chained, no back end) | 1.55 | 6.56 | 5.83 | 9.03 | 7.30 | 5.94 |
| *r*_rel [°/m] | 0.0465 | 0.0761 | 0.0225 | 0.0298 | 0.0242 | 0.0240 |

On these pairs the rerun gives the paper's own estimates to within 2e-7 (largest difference of any matrix entry) and every
number of the table above is the paper's.

The paper compares DRG with FPFH + TEASER++, APDGICP (the 4DRadarSLAM front end) and the retrained
learned methods RaFlow and CAO-RONet. Leaving aside the learned methods' within-sequence test splits,
DRG has the lowest mean TE on five of the six sequences and the lowest median TE on all six. The
exception is garden, whose keyframes are ~1 s apart, so the fusion weight is only 0.08. Trained on
the first part of a loop and tested on its last 20 %, CAO-RONet beats DRG in translation on the
three loops. In rotation, DRG matches the best geometric method once the few gap-spanning pairs
(sensor dropouts) are set aside.

**Most of the translation gain comes from the Doppler fusion (stage 6), not from a better
registration.** At 12 Hz the fused translation is 93 % Doppler, and the same fusion applied to
APDGICP gives the same accuracy.

<p align="center"><img src="docs/figures/trajectories.png" width="900"></p>
<p align="center"><em>Fig. 4 of the paper. Chained scan-to-scan estimates of every method on the longest handcart sequence (nyl) and the three vehicle sequences, with no keyframing, smoothing or loop closure. The chain is restarted where a scan falls in a short gap of the ground truth (loop1: 21 times, loop2: 9, loop3: 5, nyl: never), so the drift inside a segment is front-end drift alone. Methods that diverge leave the panel. Rerunning this repository reproduces the DRG segments exactly (<code>results/trajectories/&lt;sequence&gt;/stamp_est_traj_segmented.txt</code>).</em></p>

### Trajectory error, with and without loop closure (not in the paper)

The paper reports no absolute trajectory error. Here are DRG's whole-run trajectories, each a single chain
that is never restarted, alone and as the front end of the full
[4DRadarSLAM](https://github.com/zhuge2333/4DRadarSLAM) system run from the rosbags with loop closure
off and on. ATE is the RMSE [m] of `evo_ape tum <ground truth> <estimate> -va` (SE(3) Umeyama
alignment). The saved results (`--save_results`) and plots (`--save_plot`) of every cell are in
[`results/`](results/README.md).

| ATE [m], `evo_ape -va` | cp | garden | nyl | loop1 | loop2 | loop3 |
|---|---|---|---|---|---|---|
| ground-truth path (every scan) | 240 m | 332 m | 1,009 m | 6,944 m | 4,793 m | 4,233 m |
| DRG odometry, every scan | 1.61 | 3.15 | 9.66 | 538.74 | 121.86 | 58.38 |
| DRG odometry, at the SLAM keyframes | 1.57 | 3.15 | 9.40 | 536.44 | 120.01 | 56.85 |
| + 4DRadarSLAM back end, no loop closure | 1.66 | 7.09 | 9.35 | 535.14 | 111.72 | 55.41 |
| + 4DRadarSLAM back end, loop closure (best run) | 0.51 | 6.30 | 7.50 | 535.15 | 14.09 | 11.85 |
| loop closures (best run) | 1 | 1 | 0 | 0 | 47 | 55 |
| loop-closure runs (median ATE [m]) | 1 (0.51) | 1 (6.30) | 1 (7.50) | 1 (535.15) | 3 (57.26) | 7 (20.52) |

With loop closure the result varies from run to run, because 4DRadarSLAM's back end is not deterministic. The table shows the best run (lowest ATE); its files are in `results/slam_backend/<seq>/` and every other run is in `repeats/`. loop3, 7 runs: 11.98, 21.23, 21.63, 20.52, 11.88, 11.85, 21.45 m (median 20.52 m); loop2, 3 runs: 14.09, 57.26, 101.11 m (median 57.26 m). A rerun lands somewhere in this range, not necessarily on the best run. nyl closes no loop in either run (0 loop closures), so the difference between its two back-end rows is this run-to-run variation too. garden: the whole garden run, every scan (the paper's 1 s keyframes cover 142 m of it: ATE 1.02 m). The two odometry rows differ only in where they are scored, at every scan or at the SLAM keyframes. The back end without loop closure still optimises a pose graph with IMU and ego-velocity preintegration edges.

<p align="center"><img src="docs/figures/trajectories_whole_run.png" width="900"></p>

**Fig. 4 and the whole-run trajectories.** As in the paper, a scan pair is scored only if both of its scans lie within 0.1 s of a ground-truth sample. The ground truth is sampled at 10 Hz, but it has short gaps of up to 0.38 s, and a scan near the middle of such a gap can be more than 0.1 s from the nearest sample. This happens at 21 places on loop1, 9 on loop2 and 5 on loop3. Fig. 4 restarts the chained trajectory at each of these places, so each segment shows only the drift accumulated within it (`evo_ape -va` of the segmented trajectories: 20.7 m on loop1, 52.9 m on loop2). The whole-run trajectory is a single chain from the first scan to the last, so the drift of all segments adds up. As the paper notes, over long chains the absolute trajectory error mainly reflects where the heading error accumulates; the paper therefore scores the chained estimates by the KITTI relative errors (Table 3). On loop1 and loop2, two effects dominate the whole-run error:

* **loop1.** The rosbag has a 17.4 s radar dropout in which the car drives 90 m and turns 43°. DRG, like any scan-to-scan method, estimates 0.46 m and 0° for that pair, and the resulting heading error persists for the rest of the run. loop1 never returns to a place it has passed, so no loop closure can correct it (0 loop closures), and 4DRadarSLAM's pose graph follows the odometry rather than the IMU.
* **loop2.** There is no dropout, but the per-pair heading errors add up to 35° over 4.8 km. With loop closure, 4DRadarSLAM corrects most of it: 47 loop closures bring the ATE to 14.1 m (14.1, 57.3, 101.1 m in three runs). The result hinges on whether the loop verification accepts the first revisit. It did in 2 of these 3 runs; in the remaining run it rejected that revisit and accepted one false loop instead (304 m apart in the ground truth). With the odometry used before this rerun it was rejected and no loop closed.

Details: [results/README.md](results/README.md#odometry-trajectories) and [results/slam_backend/README.md](results/slam_backend/README.md).

<p align="center"><img src="docs/figures/loop1_loop2_segments_vs_whole_run.png" width="820"></p>

### SNAIL-Radar (not in the paper)

The same code on five sequences of [SNAIL-Radar](https://snail-radar.github.io) with two radars: radar-only scan-to-scan odometry over the whole run, ATE [m] of `evo_ape -va`. The first two rows use the paper's parameters unchanged. The third re-tunes the two gate thresholds to the ARS548 with the paper's own rules (2 dB and 0.05 m/s instead of 1 dB and 0.01 m/s), because the ARS548 quantises RCS to 1 dBsm and has a noisier Doppler, so the paper's gate kept only 34 % of its true matches:

| | 20240113_1 | 20240113_3 | 20240115_2 | 20240123_2 | 20240123_3 |
|---|---|---|---|---|---|
| ground-truth path | 541 m | 4,609 m | 6,627 m | 8,525 m | 2,228 m |
| Oculii Eagle, accumulative cloud | 48.75 | 79.66 | 71.90 | 197.87 | 45.76 |
| Continental ARS548 (Soft-REVE, as in the paper) | 5.02 | 369.52 | 805.51 | 183.99 | 121.42 |
| Continental ARS548 (Soft-REVE, gate re-tuned to the ARS548) | 4.06 | 163.08 | 155.12 | 77.95 | 37.64 |

Other inputs, the gate measurement behind the third row, 4DRadarSLAM on the ARS548 without and with loop closure, and why the ground truth is not included: [results/snail_radar/README.md](results/snail_radar/README.md).

## 🍎 Tutorial (Python)

Open 📂 [`example/example.ipynb`](example/example.ipynb) for a step-by-step walk-through.
[`example/data`](example/data) (3 MB) holds 31 consecutive scans of NTU4DRadLM *loop3*, a car at
7.6 m/s. They are exactly the clouds the paper registered for its windowed set *loop3 set 4*. The
notebook covers:

- **A 4D radar scan**: x, y, z, RCS and Doppler of every return
- **Stage 1, Soft-REVE**: ego-velocity from Doppler, residuals and confidence weights
- **Stage 2**: the 37-D descriptor
- **Stages 3–4**: mutual nearest neighbours and the Doppler–RCS gate, with how often RCS repeats between frames
- **Stages 5–6**: geometric and Doppler translations, the fusion, and the error against the ground truth
- **The whole window**: 30 pairs, checked against the paper's estimates (they agree to ~1e-10 m)

```bash
pip3 install -r requirements.txt notebook
cd example && jupyter notebook example.ipynb
```

The same check from the command line (about 1 min):

```bash
python3 scripts/reproduce_windowed_sets.py --datasets example/datasets.yaml --sets loop3_set4_5692
# 30 pairs compared with the paper: ... max |dT| = 3.10e-11   loop3  30  0.0440  0.133   PASSED
```

## 🤖 Reproduce with Claude Code

[`AGENTS.md`](AGENTS.md) is the instruction file for coding agents. It takes an agent from a fresh
machine to the paper's Tables 1 and 3, step by step, with the expected output of every command, and
it opens with the prompt that hands it to [Claude Code](https://code.claude.com/docs/en/overview).

1. Start Claude Code (`claude` in a terminal) in an empty folder or in a clone of this repository.
2. Paste the prompt from [`AGENTS.md`, section "Prompt for Claude Code"](AGENTS.md#prompt-for-claude-code).
   Its short form is:

   ```text
   Clone https://github.com/minhduc251101/DRG.git if this folder is not already a clone of it,
   then read AGENTS.md and follow it to reproduce the DRG paper's results on this machine.
   ```

3. Answer its questions (what to run, which Python environment, where NTU4DRadLM is, where to write
   the results), or append the answers to the prompt to skip them.

What it can run is listed in `AGENTS.md`, section 0: from the install check without any dataset
(level A, a few minutes) to Tables 1 and 3 from the released estimates (B), a rerun of DRG on every
scan pair (C) and the ROS node on a rosbag (D). At the end, Claude Code reports which level it reached
and compares every result with its expected value. Other coding agents can be given the same prompt.

## Installation

**Dependencies.** Python ≥ 3.8 with NumPy, SciPy, PyYAML and **Open3D 0.18.0** (the version used for
the paper). ROS Noetic is needed for the node, and [evo](https://github.com/MichaelGrupp/evo) for the
trajectory metrics. Tested on Ubuntu 20.04 with ROS Noetic (Python 3.8) and, without ROS, with
Python 3.11.

```bash
# 1. clone into a catkin workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/minhduc251101/DRG.git

# 2. Python dependencies
pip3 install -r DRG/requirements.txt

# 3. build (the ROS package is called drg)
cd ~/catkin_ws && catkin_make        # or: catkin build drg
source devel/setup.bash
```

Without ROS, step 2 is enough: the library, the scripts and the notebook run from the repository
folder (or `pip3 install -e .` installs the `drg` package).

**Python API.**

```python
from drg import DRG, DRGParams, NormStats

drg = DRG(DRGParams(), NormStats.from_yaml("config/norm_stats/loop3.yaml"))

# a scan is an (N, 5) array: x, y, z [m], RCS [dB], Doppler [m/s] (raw range rate, not ego-compensated)
target = drg.prepare(scan_k)                               # stages 1-2, once per scan
source = drg.prepare(scan_k1)
result = drg.register(source, target, dt=t_k1 - t_k)       # stages 3-6

result.T              # 4x4, maps source points into the target frame
result.fusion_weight  # the weight of Eq. 8 (0.93 at 12 Hz)
result.n_gated        # correspondences kept by the Doppler-RCS gate
```

## Run on a rosbag (ROS Noetic)

**Data.** Download a sequence of [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM), e.g.
`loop3`: `loop3_2022-06-03_{0,1,2}.bag` and `gt_odom.txt`. DRG reads the radar topic
`/radar_enhanced_pcl` (`sensor_msgs/PointCloud`, channels `Doppler` and `Power` = RCS).

**One command** starts the node and RViz and plays the bags:

```bash
roslaunch drg ntu4dradlm.launch sequence:=loop3 output_dir:=$HOME/drg_out/loop3 \
    bags:="$(ls /data/NTU4DRadLM/loop3/*.bag | xargs)"
```

| Argument | Default | Meaning |
|---|---|---|
| `sequence` | `cp` | selects `config/norm_stats/<sequence>.yaml`: `cp`, `garden`, `nyl`, `loop1`, `loop2`, `loop3` |
| `bags` | – | bag files separated by spaces, played in this order |
| `rate` | `0.05` | playback rate (see *Speed* below) |
| `ransac_trials` | `40` | paper value. `1` gives the same estimates about 13× faster |
| `output_dir` | – | where `stamp_est_traj.txt` and `relative_poses.txt` are written |
| `rviz` | `true` | show the trajectory and the registered scans (coloured by RCS) |
| `play_args` | – | extra `rosbag play` options, e.g. `play_args:="-s 60 -u 20"` for a 20 s test from 60 s |

**Or step by step**, with your own `rosbag play`:

```bash
roscore
roslaunch drg drg.launch norm_stats:=$(rospack find drg)/config/norm_stats/loop3.yaml \
    output_dir:=$HOME/drg_out/loop3
rosbag play --clock -r 0.05 /data/NTU4DRadLM/loop3/loop3_2022-06-03_{0,1,2}.bag --topics /radar_enhanced_pcl
```

**Speed.** `rosbag play` first indexes the bags, about 1 min per 3 GB bag file, before the first
scan arrives. DRG is a single-threaded Python implementation and takes about 1–5 s per scan pair with
the paper's 40 RANSAC trials (0.4 s with `ransac_trials:=1`), against an 83 ms frame period. The node
**never drops a scan**, because it registers consecutive scans; it only falls behind. Play the bags
slowly or let the node work through its queue. It logs `[DRG] scan N` for every scan and flushes
the output files after every pair.

**Topics and frames.**

| Topic | Type | |
|---|---|---|
| `/radar_enhanced_pcl` (input, remap with `input_topic:=`) | `sensor_msgs/PointCloud` | `cloud_type:=PointCloud2` for PointCloud2 input |
| `/drg_odometry/odometry` | `nav_msgs/Odometry` | radar pose in `odom` |
| `/drg_odometry/path` | `nav_msgs/Path` | |
| `/drg_odometry/cloud_registered` | `sensor_msgs/PointCloud2` | current scan in `odom`, intensity = RCS |
| TF `odom → radar_body` | | |

The raw radar points are rotated into the body axes of 4DRadarSLAM (`rotate_to_body:=true`, a 3.13°
rotation; the translation is dropped). The axes are then parallel to the Livox / ground-truth frame
and the origin stays at the radar, as in the paper. Set `rotate_to_body:=false` for other radars and
for clouds that are already rotated.

**The paper's exact input.** The paper did not register the raw scans. It registered the output of
the preprocessing of [4DRadarSLAM](https://github.com/zhuge2333/4DRadarSLAM) (its
[Go-RIO](https://github.com/wooseongY/Go-RIO) version), topic `/filtered_points`: the raw points
rotated into the body axes, range and radius-outlier filtered, without the scans in which the
platform stands still. The paper's per-scan dumps store exactly these clouds (checked field by field
on nyl). With 4DRadarSLAM running on the same bag, DRG registers the same input as in the paper:

```bash
roslaunch drg drg.launch input_topic:=/filtered_points cloud_type:=PointCloud2 rotate_to_body:=false \
    norm_stats:=$(rospack find drg)/config/norm_stats/loop3.yaml output_dir:=$HOME/drg_out/loop3_paper
```

Registering the raw scans instead changes little: on the same 7,609 loop3 pairs the mean TE goes
from 0.0778 m (paper input) to 0.0776 m, and the mean RE changes in the fourth decimal (measured
offline, with normalisation statistics from the raw scans). The other sequences were not measured
this way.

## Evaluate against ground truth

The node and `scripts/register_dump.py` write two files:

* `stamp_est_traj.txt`: the chained trajectory P<sub>k+1</sub> = P<sub>k</sub> T<sub>k</sub> in TUM format
* `relative_poses.txt`: one line per pair: `stamp_target stamp_source` + the top 3×4 of **T**

**The paper's metrics** (Tables 1 and 3):

```bash
# per-pair TE / RE (use --max-gt-gap 0.15 on nyl)
python3 scripts/evaluate_pairs.py --est ~/drg_out/loop3/relative_poses.txt --gt /data/NTU4DRadLM/loop3/gt_odom.txt

# KITTI relative drift t_rel / r_rel over 100-800 m segments
python3 scripts/evaluate_trajectory.py --est ~/drg_out/loop3/relative_poses.txt --gt /data/NTU4DRadLM/loop3/gt_odom.txt
```

**With [evo](https://github.com/MichaelGrupp/evo).** The ground truth is sampled at 10 Hz and the
radar at 12 Hz. evo's nearest-stamp matching (`--t_max_diff`) can be up to 50 ms off, which is up to
0.4 m at car speed. Interpolate the ground truth at the radar stamps first, as the paper does:

```bash
cd ~/drg_out/loop3
python3 ~/catkin_ws/src/DRG/scripts/interpolate_gt.py --gt /data/NTU4DRadLM/loop3/gt_odom.txt \
    --est stamp_est_traj.txt --out stamp_gt_traj.txt

# absolute trajectory error after SE(3) alignment, with evo's saved results and plots
evo_ape tum stamp_gt_traj.txt stamp_est_traj.txt -va --save_results ape.zip --save_plot ape.pdf --plot_mode xy
# relative pose error between consecutive scans (close to the paper's per-pair TE)
evo_rpe tum stamp_gt_traj.txt stamp_est_traj.txt -va --delta 1 --delta_unit f --pose_relation trans_part
```

On the tutorial window, interpolation lowers the APE RMSE from 0.23 m (nearest stamp) to 0.050 m.
`evo_rpe` gives a mean of 0.0442 m against the paper's 0.0440 m; the two differ only in how the error
transform is composed.

**The released trajectories.** [`results/trajectories/<sequence>/`](results/README.md#odometry-trajectories)
holds DRG's trajectory over the whole run (the estimates of every consecutive pair, chained from the
identity without restarting, like the node's own output), the ground truth at the same stamps, a plot,
and the output of `evo_ape -va` with its saved results and plots (`evo_ape.txt`, `.zip`, `.pdf`):

```bash
cd results/trajectories/loop3
evo_ape tum stamp_gt_traj.txt stamp_est_traj.txt -va --save_results evo_ape.zip --save_plot evo_ape.pdf --plot_mode xy
```

`bash scripts/evo_ape_all.sh` reruns this for every trajectory in `results/`.

## Reproducing the paper result

The paper's results are DRG's estimates of every consecutive scan pair, scored against the ground truth
(Tables 1 and 3, and the trajectories of Fig. 4). There are three ways to get them.
[`AGENTS.md`](AGENTS.md) goes through all three step by step, with the expected output of every command.
To let Claude Code follow it for you, use the prompt at the top of `AGENTS.md`, see [Reproduce with Claude Code](#-reproduce-with-claude-code).

**(a) Run DRG on every scan pair, as the paper did.** The paper registers the per-scan clouds that
4DRadarSLAM (Go-RIO version) writes with its `/radar_graph_slam/dump` service (one folder per scan, with
`cloud.pcd` and the stamp). Setting the keyframe thresholds to zero
(`keyframe_delta_trans_back_end:=0 keyframe_delta_angle:=0`) keeps every scan; garden uses the
default ~1 s keyframes. List the dumps and the ground truth in `config/datasets.yaml`, then:

```bash
cp config/datasets_example.yaml config/datasets.yaml     # edit the paths
python3 scripts/reproduce_paper.py --datasets config/datasets.yaml --jobs 16 --out rerun
```

`reproduce_paper.py` registers every consecutive pair of each dump with the paper's parameters, in
parallel (every pair is seeded, so the estimates do not depend on `--jobs`), writes
`rerun/relative_poses/`, and prints the DRG rows of Tables 1 and 3 (the table in [Results](#-results))
with the largest difference to the released estimates. The six sequences take about 2 h with 16
processes. `results/` was made the same way: `scripts/register_dump.py`, which calls the same
registration function, on all 59,621 pairs in chunks of 200 pairs run in parallel (and the
2 cp pairs that span a scan without point cloud, registered with the same library). The
trajectories, the version of Fig. 4 and their `evo_ape -va` results then follow with

```bash
python3 scripts/export_trajectories.py --datasets config/datasets.yaml --est-dir rerun/relative_poses \
    --out-dir rerun/trajectories --segmented
cd rerun/trajectories/loop1
evo_ape tum stamp_gt_traj.txt stamp_est_traj.txt -va --save_results evo_ape.zip --save_plot evo_ape.pdf --plot_mode xy
evo_ape tum stamp_gt_traj.txt stamp_est_traj_segmented.txt -va      # the version of Fig. 4
```

Smaller checks on the same dumps:

```bash
# one window, e.g. carpark tuning set 1 (30 pairs from frame 4470)
python3 scripts/register_dump.py --dump /data/dumps/cp --stats config/norm_stats/cp.yaml \
    --gt /data/NTU4DRadLM/cp/gt_odom.txt --start 4470 --num-pairs 30 --out results/cp_set1

# all 37 windowed sets, compared pair by pair with the paper's estimates
python3 scripts/reproduce_windowed_sets.py --datasets config/datasets.yaml
```

**(b) Re-score the released estimates (seconds).** `results/relative_poses/<seq>.txt` holds DRG's
estimate of every consecutive pair, from the run above. Scored as in the paper (only the pairs whose two
frames have ground truth), they give Tables 1 and 3; this needs only the ground truth:

```bash
python3 scripts/reproduce_paper.py --datasets config/datasets.yaml --score-only --out results
```

```
sequence    pairs  scored  mean TE  med TE  mean RE  med RE  t_rel %   r_rel   largest difference to results/relative_poses
cp           5256    5256   0.0135  0.0104    0.189   0.156     1.55  0.0465   (the released file itself)
garden        272     272   0.0554  0.0459    0.555   0.473     6.56  0.0761   (the released file itself)
nyl         11243   11243   0.0229  0.0164    0.223   0.188     5.83  0.0225   (the released file itself)
loop1       16038   15992   0.0736  0.0487    0.101   0.062     9.03  0.0298   (the released file itself)
loop2       11587   11568   0.0687  0.0447    0.142   0.084     7.30  0.0242   (the released file itself)
loop3        7619    7609   0.0778  0.0532    0.114   0.084     5.94  0.0240   (the released file itself)

TE [m] and RE [deg] per pair (Table 1); t_rel [%] and r_rel [deg/m] over 100-800 m (Table 3).
```

One sequence at a time: `scripts/evaluate_pairs.py` (Table 1) and `scripts/evaluate_trajectory.py`
(Table 3), with `--max-gt-gap 0.15` on nyl:

```bash
python3 scripts/evaluate_pairs.py      --est results/relative_poses/loop3.txt --gt /data/NTU4DRadLM/loop3/gt_odom.txt
python3 scripts/evaluate_trajectory.py --est results/relative_poses/loop3.txt --gt /data/NTU4DRadLM/loop3/gt_odom.txt
```

`results/trajectories/` holds the same estimates chained into trajectories for evo, with the ground
truth at the same stamps. `scripts/export_trajectories.py` rebuilds them, `scripts/evo_ape_all.sh`
scores them with `evo_ape -va` (saving the results and plots) and `scripts/plot_trajectories.py` plots
them. With `--segmented`, the first and the last also rebuild the segmented version drawn in Fig. 4
(restarted where a scan falls in a gap of the ground truth)
([results/README.md](results/README.md#odometry-trajectories)).

**(c) From the rosbag.** See [Run on a rosbag](#run-on-a-rosbag-ros-noetic). On the raw
`/radar_enhanced_pcl` scans the results differ slightly from the paper's (see above). On 4DRadarSLAM's
`/filtered_points` the node registers the paper's input.

**The SLAM results (not in the paper)** are reproduced step by step in [results/slam_backend/README.md](results/slam_backend/README.md#reproduce): the DRG odometry table (`scripts/slam/build_odom_table.py`), the 4DRadarSLAM configuration (`config/4dradarslam/`, checked by `scripts/slam/check_launch.py`), the `/odom` publisher and the scoring.

### Verification

* **Full rerun.** Run again on every consecutive pair of the six sequences (52,015
  pairs, plus garden every scan), this code reproduces the paper's own estimates of its 51,940 pairs to
  within 2e-7 (largest difference of any matrix entry). Re-scored with `evaluate_pairs.py` and
  `evaluate_trajectory.py`, they give the paper's mean / median TE and RE (Table 1) and *t*_rel / *r*_rel
  (Table 3) on all six sequences, to every printed digit.
* **Library.** `reproduce_windowed_sets.py` reruns all 37 windowed sets (1,110 pairs) with Open3D 0.18.0.
  690 pairs are bit-identical to the paper's estimates and all agree within 2.5e-10 m, which is
  floating-point rounding (`tests/reference/windowed_sets_seed42.json`).
* **ROS node.** In ROS Noetic (Python 3.8), the node fed the tutorial window as 4DRadarSLAM-style
  `/filtered_points` messages reproduces the paper's 30 estimates to 3e-11 m. On a slice of the raw
  loop3 bag, played through `ntu4dradlm.launch`, it matches the library on the same scans to 2e-12 m.
* **Unit tests.** `python3 -m pytest tests/` runs on synthetic scans and on the tutorial window.

## Parameters and how they were tuned

All parameters are in [`config/drg.yaml`](config/drg.yaml). **[docs/PARAMETERS.md](docs/PARAMETERS.md)**
explains each one: what it does, which values were tried, the results on the tuning sets, and how to
adapt DRG to another radar. In short:

| Parameter | Value | Purpose | How it was set |
|---|---|---|---|
| `tau_rcs` | 1 dB | gate: max RCS difference of a match | radar property: RCS repeats bit-exactly for 85–88 % of true matches. Tuned: 0.5 dB gave the same result |
| `tau_residual` | 0.01 m/s | gate: max Doppler-residual difference | 3–10 × the median of true matches. Tuned: 0.005 m/s gave the same result |
| `inlier_distance` | 0.3 m | RANSAC inlier distance = IRLS truncation | 3 × the 0.1 m FPFH voxel. Tuned: 0.2 m gave the same result |
| `cauchy_c` | 0.1 m | Cauchy scale of the IRLS | tuned: 0.05 m changed TE by ≤ 0.1 mm |
| `sigma_v` | 0.10 m/s | ego-velocity accuracy (fusion) | not tuned: literature (RAVE: 0.04–0.14 m/s) |
| `sigma_g` | 0.03 m | geometric translation accuracy (fusion) | not tuned: per-pair geometric error on the tuning sets |
| `radar_block_weight` | 0.5 | weight of the radar channels in the descriptor | not tuned: equal weighting |
| `ransac_trials` | 40 | RANSAC restarts | not tuned. 1 trial gives the same estimates, ~13× faster |

**Protocol.** All development decisions were made on five windowed sets of 30 pairs: carpark sets
1–3 (12 Hz) and garden sets 1–2 (1 s keyframes). The design choices (descriptor, matching, robust
loss, gate, refinement, fusion) were compared on carpark set 1 and garden set 1. Two of them
mattered: explicit mutual-NN correspondences with correspondence RANSAC (instead of feature-matching
RANSAC) and the Doppler fusion. The four tuned constants were varied on the two garden sets, where
geometry carries the translation. **DRG is insensitive to all of them**: no variant changed the mean
TE by more than 0.3 mm, so the starting values were kept and frozen. The other 32 windowed sets and all 51,940 pairs, including the car
sequences at 5–7 m/s, were then run once with these values. The fusion weight is not tuned: it
follows from Δt alone, ω = 1 / (1 + (σ<sub>v</sub> Δt / σ<sub>g</sub>)²).

## Repository layout

```
DRG/
├── src/drg/                     # the algorithm (pure Python; no ROS needed)
│   ├── params.py                #   every constant, with its meaning
│   ├── ego_velocity.py          #   stage 1    Soft-REVE
│   ├── descriptor.py            #   stage 2    37-D descriptor + normalisation statistics
│   ├── correspondence.py        #   stages 3-4 mutual NN + Doppler-RCS gate
│   ├── solver.py                #   stage 5    RANSAC + Cauchy IRLS
│   ├── fusion.py                #   stage 6    Doppler translation fusion
│   ├── registration.py          #   DRG class: prepare(scan) + register(source, target, dt)
│   ├── radar_frames.py          #   NTU4DRadLM radar -> body rotation
│   ├── ros_conversions.py       #   PointCloud / PointCloud2 -> numpy
│   ├── io.py, sequence.py       #   4DRadarSLAM dump reader, pair loop
│   └── evaluation.py            #   GT interpolation, TE / RE, KITTI drift, TUM output
├── nodes/drg_odometry_node.py   # ROS1 node
├── launch/                      # drg.launch (node), ntu4dradlm.launch (node + rosbag play)
├── config/
│   ├── drg.yaml                 # the paper's parameters
│   ├── norm_stats/<seq>.yaml    # descriptor normalisation statistics per sequence (+ snail_radar/)
│   ├── 4dradarslam/             # the 4DRadarSLAM back-end configuration of results/slam_backend
│   └── snail_radar/             # the parameter changes used on SNAIL-Radar (1 RANSAC trial; ARS548 RCS floor; ARS548 gate)
├── example/                     # tutorial notebook + 31 scans of loop3 + ground truth
├── scripts/                     # offline registration, evaluation and reproduction
│   ├── slam/                    #   DRG as the front end of 4DRadarSLAM: odometry table, /odom publisher, scoring
│   └── snail_radar/             #   SNAIL-Radar scans and ground truth -> DRG input
├── results/
│   ├── relative_poses/          # DRG's estimate of every consecutive scan pair (rerun of this code)
│   ├── trajectories/            # the same, chained over the whole run + ground truth + evo_ape -va results
│   ├── slam_backend/            # not in the paper: DRG in 4DRadarSLAM, without / with loop closure
│   └── snail_radar/             # not in the paper: DRG on SNAIL-Radar (Oculii Eagle, ARS548)
├── tests/                       # unit tests + the paper's estimates of the 37 windowed sets
└── docs/PARAMETERS.md           # what every parameter does and how it was chosen
```

## Notes and limitations

* **Scan-to-scan odometry drifts.** Without a back end the chained trajectory is not globally
  consistent: the ATE over the whole run is 122 m on loop2 (4.8 km). Like any
  scan-to-scan method DRG sees nothing between two scans: across loop1's 17.4 s radar dropout it estimates
  0.46 m while the car drives 90 m and turns 43° ([results](results/README.md#odometry-trajectories)).
* **Runs offline.** About 1–5 s per pair with 40 RANSAC trials and 0.4 s with one trial, on one CPU core.
  Real time at 12 Hz needs a compiled implementation.
* **Normalisation statistics** of the descriptor are computed per sequence from every 10th scan
  (`scripts/compute_norm_stats.py`), as in the paper. They include future scans, so the evaluated
  pipeline is not causal. Using another sequence's statistics changed the mean TE by at most 0.14 mm.
* **Doppler convention.** DRG expects the raw signed range rate (negative = approaching) and fits
  d = −n·v. The Doppler prior only helps at frame rate: at 1 s keyframes DRG reduces to its geometric
  back end.
* **Radar timestamps.** The stamp intervals of this radar alternate long and short. The ground truth is
  interpolated at these stamps and **t**_dop uses the same Δt, which can favour DRG in per-pair TE
  (paper, Sec. 4.1).
* **One radar.** The evidence covers one sensor (Oculii Eagle). The 1 dB RCS threshold depends on its
  RCS repeatability; re-tune it for another radar ([docs/PARAMETERS.md](docs/PARAMETERS.md#adapting-drg-to-another-radar)).
* The only failure check is a plausibility test (|**t**| ≤ 5 m, roll and pitch ≤ 10°). If every
  hypothesis fails, DRG returns the identity and the node logs `FAILED`.

## Dataset

The experiments use the [**NTU4DRadLM**](https://github.com/junzhang2016/NTU4DRadLM) 4D radar dataset.
Many thanks to Jun Zhang *et al.* for releasing it. The tutorial data in `example/data` are a short
excerpt of its *loop3* sequence, after 4DRadarSLAM preprocessing.

## Citation

```bibtex
@inproceedings{nguyen2026drg,
  title     = {{4D} Radar Scan-to-Scan Registration using Doppler Translation Fusion},
  author    = {Nguyen, Van Minh Duc and Li, Tiancheng and Li, Zhenzi and Alempijevic, Alen and Huang, Shoudong},
  booktitle = {Proceedings of the Australasian Conference on Robotics and Automation (ACRA)},
  year      = {2026}
}
```

## Acknowledgements

We thank the authors of [NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM),
[4DRadarSLAM](https://github.com/zhuge2333/4DRadarSLAM), [REVE](https://github.com/christopherdoer/reve),
[Open3D](http://www.open3d.org) and [evo](https://github.com/MichaelGrupp/evo).

## License

[MIT](LICENSE)
