# DRG — setup guide for coding agents

You are setting up and running DRG for a user. DRG registers consecutive 4D radar scans using their Doppler velocity and radar cross section (RCS), with no training. It is the method of the ACRA 2026 paper *4D Radar Scan-to-Scan Registration using Doppler Translation Fusion*. This repository releases it as a pure-Python library and a ROS 1 node, together with the scripts and estimates behind the paper's results.

Take the user from a fresh machine to the paper's DRG results on the six NTU4DRadLM sequences: the per-pair errors of Table 1 and the drift of Table 3. Follow the steps in order. Run every command yourself, check its output before moving on, and stop to ask the user only where a step says **Ask**. Every expected output below was checked on a fresh clone of this repository (25 Nov 2026), except where a step says otherwise.

## Prompt for Claude Code

*This section is for the person who hands this file to an agent. The agent starts at section 0.*

Start [Claude Code](https://code.claude.com/docs/en/overview) (`claude` in a terminal) in an empty folder or in a clone of this repository, and paste:

```text
Clone https://github.com/minhduc251101/DRG.git and work in the clone
(skip this if the current folder is already a clone of it).
Then follow its AGENTS.md to reproduce the DRG paper's results on this machine:
- First ask me the questions of section 0 that this message does not answer,
  and wait for my answers.
- Run the steps in order, and run every command yourself. Before you move on,
  compare the output with the expected output in AGENTS.md.
  If a check fails, stop and show me both.
- Never change parameters, package versions or the scoring to make a number match.
  Do not edit files that git tracks, and ask me before you use sudo.
- End with the report of section 11.
```

Claude Code then asks the questions of section 0: what to run, which Python environment to use, where NTU4DRadLM is and where to write the results. To skip them, add the answers at the end of the prompt, for example `Run A and B. Make a new venv with python3.11. NTU4DRadLM is in /data/NTU4DRadLM.` **A** always comes first; B, C and D are independent of each other:

| | What runs | What you need |
|---|---|---|
| **A** | The unit tests, then one window of the paper's input (31 scans of loop3, shipped in `example/`), compared with the paper's estimates. A few minutes. | Python 3.8–3.11 and about 2 GB of disk. No dataset. |
| **B** | Tables 1 and 3, recomputed from DRG's released estimates. Under a minute. | The six `gt_odom.txt` files of NTU4DRadLM (about 6 MB). |
| **C** | DRG on every scan pair, as in the paper. About 2 h with 16 processes. | The per-scan dumps (about 21 GB), made with ROS Noetic and a pinned Go-RIO from the bags (49 GB). |
| **D** | The ROS node on a rosbag, starting with a 10 s test on loop3. | ROS Noetic on Ubuntu 20.04, and the bags. |

At the end it reports which level it reached, compares every result with its expected value and lists any difference (section 11). Other coding agents can be given the same prompt.

## 0. Ask the user first

**Ask**, in a single message:

1. **What to run.** A always comes first. B, C and D are independent of each other:
   - **A. Check the install.** A few minutes and no dataset. Runs the unit tests, then registers one window of the paper's input (31 scans of loop3, shipped in `example/`) and compares it with the paper's estimates.
   - **B. Recompute Tables 1 and 3.** Under a minute. Scores DRG's released estimate of every scan pair (`results/relative_poses/`). Needs only the six ground-truth files `gt_odom.txt` of NTU4DRadLM.
   - **C. Rerun DRG on every scan pair.** About 2 h with 16 processes. Needs the per-scan dumps that the paper registered (about 21 GB). They are not in this repository. Making them needs ROS Noetic, a pinned version of Go-RIO's 4DRadarSLAM and the rosbags (step 6), and a new dump can differ slightly from the paper's.
   - **D. Run the ROS node on a rosbag.** Needs ROS Noetic on Ubuntu 20.04.

   On top of these, the user can ask for the whole-run trajectories with `evo_ape` (step 9), or for DRG inside 4DRadarSLAM or on SNAIL-Radar (step 10).
2. **Python environment.** A venv or a conda environment with Python 3.8–3.11 (Open3D 0.18.0 has no wheel for 3.12 or later). For D, the system Python 3.8 of ROS Noetic.
3. **Where NTU4DRadLM is.** The folders `cp`, `garden`, `nyl`, `loop1`, `loop2` and `loop3`, each with `gt_odom.txt` (B) and the `.bag` files (C, D). For C, also where the dumps are or should go. If they do not have the dataset, point them to the download link on [github.com/junzhang2016/NTU4DRadLM](https://github.com/junzhang2016/NTU4DRadLM) (https://rebrand.ly/ntu4dradlm). B needs only the six `gt_odom.txt` files (about 6 MB). The bags of all six sequences take 49 GB.
4. **Where to write results** (default `~/drg_out`) and, for C, how many processes to use.

## 1. Check the machine

- `python3 --version` must be 3.8–3.11. Otherwise create an environment that has one, e.g. `conda create -n drg python=3.11`.
- `df -h`: about 2 GB for the clone and the Python packages. For C, about 21 GB more for the dumps, plus 49 GB for the bags they are made from.
- C: run `nproc`. DRG is single-threaded, and `reproduce_paper.py` starts one process per core by default.
- D: `ls /opt/ros` must show `noetic`.

Tested on Linux: Python 3.11 without ROS, and Ubuntu 20.04 with ROS Noetic and Python 3.8.

## 2. Get the code

```bash
git clone https://github.com/minhduc251101/DRG.git
cd DRG
```

Run every later command from this folder unless a step says otherwise.

Layout:

- `src/drg/`: the algorithm, one module per stage (pure Python, no ROS).
- `scripts/`: registration, evaluation and reproduction.
- `config/drg.yaml`: the paper's parameters.
- `config/norm_stats/<seq>.yaml`: the descriptor normalisation statistics of each sequence.
- `results/relative_poses/<seq>.txt`: DRG's estimate of every consecutive scan pair. Tables 1 and 3 are computed from these files.
- `results/trajectories/`, `results/slam_backend/` and `results/snail_radar/`: results beyond the paper.
- `example/`: the tutorial notebook and 31 scans of loop3.
- `nodes/` and `launch/`: the ROS 1 node (package `drg`).
- `tests/`: the unit tests, and the paper's estimates for its 37 windowed sets.

## 3. Install

### Without ROS (A, B, C)

```bash
python3 -m venv .venv && source .venv/bin/activate        # or: conda activate drg
python3 -m pip install --upgrade pip                        # Open3D 0.18.0 wheels need pip >= 20.3
pip install -r requirements.txt pytest
python3 -c "import open3d; print(open3d.__version__)"     # must print 0.18.0
```

Open3D must be exactly 0.18.0, as pinned in `requirements.txt`. The estimates were verified against it, and other versions may draw different RANSAC samples. The scripts run from the repository folder. `pip install -e .` is needed only to import `drg` from elsewhere.

### ROS Noetic (D)

Use the system Python 3.8, not a venv, so that ROS and the node see the same packages. If `python3 -m pip` is missing (the `osrf/ros:noetic-desktop-full` image has no pip), install it first with `sudo apt install python3-pip`.

```bash
python3 -m pip install --user --upgrade pip
python3 -m pip install --user -r requirements.txt
mkdir -p ~/catkin_ws/src && ln -s "$PWD" ~/catkin_ws/src/DRG
cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make && source devel/setup.bash
rospack find drg                                            # must print the path of the clone
cd -                                                        # back to the clone
```

## 4. Check the install (A)

```bash
python3 -m pytest tests/ -q                                 # must end with: 12 passed
python3 scripts/reproduce_windowed_sets.py --datasets example/datasets.yaml --sets loop3_set4_5692
```

The second command registers the 30 pairs of the shipped window with the paper's parameters. It then compares every estimated transform with the paper's. It takes about 30 s and ends like this:

```
30 pairs compared with the paper: 0 bit-identical, max |dT| = 3.10e-11

sequence    pairs  TE mean [m]  RE mean [deg]
loop3          30       0.0440          0.133

PASSED
```

`max |dT|` is the largest difference of any matrix entry. It must stay below the tolerance of 1e-9, which is floating-point rounding, and the last line must be `PASSED`. On another CPU, `max |dT|` and the number of bit-identical pairs can come out slightly different.

The tutorial notebook `example/example.ipynb` walks a person through the six stages on the same window (`pip install notebook`, then `cd example && jupyter notebook example.ipynb`). If A is all the user asked for, report the two results above and stop.

## 5. Recompute Tables 1 and 3 (B)

`config/datasets.yaml` tells the scripts where the data is: for each sequence, the dump (C only), the ground truth and its gap limit. The file is ignored by git.

```bash
cp config/datasets_example.yaml config/datasets.yaml
```

Set every `gt:` path to the user's `<NTU4DRadLM>/<seq>/gt_odom.txt`, and for C the `dump:` paths. The file has a seventh entry, `garden_every_scan`, which step 9 uses: it takes garden's `gt_odom.txt` too. Keep `max_gt_gap` as it is: 0.15 s on nyl and 0.1 s elsewhere, as in the paper. Then:

```bash
python3 scripts/reproduce_paper.py --datasets config/datasets.yaml --score-only --out results
```

This reads `results/relative_poses/` and writes nothing. The output must match to the last digit:

```
sequence    pairs  scored  mean TE  med TE  mean RE  med RE  t_rel %   r_rel   largest difference to results/relative_poses
cp           5256    5256   0.0135  0.0104    0.189   0.156     1.55  0.0465   (the released file itself)
garden        272     272   0.0554  0.0459    0.555   0.473     6.56  0.0761   (the released file itself)
nyl         11243   11243   0.0229  0.0164    0.223   0.188     5.83  0.0225   (the released file itself)
loop1       16038   15992   0.0736  0.0487    0.101   0.062     9.03  0.0298   (the released file itself)
loop2       11587   11568   0.0687  0.0447    0.142   0.084     7.30  0.0242   (the released file itself)
loop3        7619    7609   0.0778  0.0532    0.114   0.084     5.94  0.0240   (the released file itself)
```

These are the DRG rows of two tables:

- **Table 1:** mean and median per-pair translation error TE [m] and rotation error RE [°].
- **Table 3:** the KITTI drift of the chained estimates over 100–800 m, t_rel [%] and r_rel [°/m].

As in the paper, a pair is scored only if both of its scans have a ground-truth sample within `max_gt_gap`. That gives 51,940 scored pairs in total. Frames, units and the direction of **T** are under Conventions, near the end of this file.

To score one sequence at a time (same numbers):

```bash
python3 scripts/evaluate_pairs.py      --est results/relative_poses/nyl.txt --gt <NTU4DRadLM>/nyl/gt_odom.txt --max-gt-gap 0.15
python3 scripts/evaluate_trajectory.py --est results/relative_poses/nyl.txt --gt <NTU4DRadLM>/nyl/gt_odom.txt --max-gt-gap 0.15
```

Both scripts default to `--max-gt-gap 0.1`, but nyl needs 0.15. With 0.1, nyl scores 11,231 pairs instead of 11,243, and its t_rel is 6.49 % instead of 5.83 %.

## 6. Make the per-scan dumps (C only)

Skip this step if the user already has the six dumps.

**What the paper registered.** In the paper, DRG did not register the raw radar messages. It registered the scans after the preprocessing of 4DRadarSLAM (Go-RIO version, topic `/filtered_points`). That preprocessing rotates each scan into the body axes, applies range and radius-outlier filters, and drops the scans in which the platform stands still. 4DRadarSLAM's `/radar_graph_slam/dump` service saved these scans. A dump is a folder with one subfolder `%06d/` per scan:

- `cloud.pcd`: binary, 8 float32 fields. DRG reads x, y and z, RCS from `intensity` and Doppler from `curvature` (some dumps name these two `rcs` and `doppler`).
- `data`: DRG reads the line `stamp <sec> <nsec>`.

**Use Go-RIO commit `65cfefc` (28 May 2025).** Its radar preprocessing source, `4DRadarSLAM/apps/preprocessing_nodelet_ntu.cpp`, is identical to the copy that made the paper's nyl, loop1, loop2 and loop3 dumps. Do not use Go-RIO's `main`: commit `ed459ee` (21 Sep 2026) rewrote that preprocessing. It introduced a new nodelet, `PreprocessingNodeletNTU`, with new ego-velocity and ground-segmentation options, so its `/filtered_points` are not the paper's input.

```bash
git clone https://github.com/wooseongY/Go-RIO.git && cd Go-RIO && git checkout 65cfefc
# build its catkin workspace with ROS Noetic as its README describes (ROS package: gorio)
```

For each sequence, start the launch file below with the settings shown, then play the sequence's bags:

| sequence | launch file in `4DRadarSLAM/launch/` | settings |
|---|---|---|
| cp | `ntu_cp.launch` | `keyframe_delta_trans_back_end:=0 keyframe_delta_angle:=0` |
| garden | `ntu_cp.launch` ¹ | the default keyframes (0.5 m / 10°) ² |
| nyl | `ntu_nyl.launch` | as cp, plus `enable_dynamic_object_removal:=false` ³ |
| loop1 | `ntu_loop3.launch` ¹ | as cp |
| loop2 | `ntu_loop2.launch` | as cp, plus `radius_min_neighbors` = 5 in your copy ⁴ |
| loop3 | `ntu_loop3.launch` | as cp |

1. There is no launch file for garden or loop1. The one named has the same preprocessing.
2. The paper uses garden's ~1 s keyframes. Every other sequence keeps every scan: with both keyframe thresholds at 0, every scan that leaves the preprocessing becomes a keyframe, and so gets its own dump folder.
3. `ntu_nyl.launch` defaults it to `true`; the paper's dumps had it off.
4. This is a parameter of the preprocessing node, not a launch argument, so edit it in the file. `ntu_loop2.launch` sets it to `1`, while the other launch files and the paper's loop2 dump use `5`.

At this commit every `ntu_*.launch` also starts RViz. It also includes `rosbag_play_ntu.launch`, which plays one hard-coded bag (`/root/data/4dradarslam/NTU4Dradlm/cp/cp.bag`). Make a copy of the launch file in the same `launch/` folder, so that `roslaunch gorio <copy>` finds it, and remove that include, because you play the bags yourself below. On a headless machine, remove the RViz node too.

Then run each sequence as below, one step after the other, checking each before the next. This follows the paper's runs, which used the same playback options; in those runs, the launch file played the bags itself.

```bash
# one port and one ROS_HOME per run
export ROS_MASTER_URI=http://127.0.0.1:11411 ROS_HOME=$HOME/drg_out/ros_loop3
mkdir -p $ROS_HOME

# start 4DRadarSLAM (roslaunch also starts a master on this port), then wait until it is up
roslaunch -p 11411 gorio <copy of ntu_loop3.launch> keyframe_delta_trans_back_end:=0 keyframe_delta_angle:=0 \
    > $ROS_HOME/run.log 2>&1 &
RL=$!
until rosnode list 2>/dev/null | grep -q radar_graph_slam_nodelet; do sleep 1; done

# play the bags; this returns when they have finished
rosbag play --clock --rate=1 --delay=5 <NTU4DRadLM>/loop3/loop3_2022-06-03_{0,1,2}.bag \
    --topics /radar_enhanced_pcl /vectornav/imu

# the back end is still working through its queue: wait until its log has not grown for 3 minutes
prev=-1
while [ "$(stat -c %s $ROS_HOME/run.log)" != "$prev" ]; do
  prev=$(stat -c %s $ROS_HOME/run.log); sleep 180
done

# dump (prints "success: True"), count the scan folders, then stop 4DRadarSLAM and its master
rosservice call /radar_graph_slam/dump "destination: '/data/dumps/loop3'"
find /data/dumps/loop3 -mindepth 1 -maxdepth 1 -type d -name '[0-9]*' | wc -l
kill -INT $RL; wait $RL
```

To run several sequences at once, give each run its own port and `ROS_HOME`. Stop each one by its own PID, as above: a `pkill -f` pattern can also stop the other runs, or the shell you are working in.

**Check every dump against the paper's.** Count the scan folders in each:

| | cp | garden | nyl | loop1 | loop2 | loop3 |
|---|---|---|---|---|---|---|
| scan folders | 5,261 | 273 | 11,244 | 16,039 | 11,588 | 7,620 |

In the paper's cp dump, two of these folders (`003247` and `003312`) have no `cloud.pcd`. The first two columns of `results/relative_poses/<seq>.txt` hold the stamps of the paper's scans, so you can compare a new dump scan by scan. Report every difference to the user.

**How close a new dump comes.** These settings have not been run end to end from a fresh Go-RIO clone. They are the settings of the paper's dumps, compared file by file with commit 65cfefc. On our machine, the same preprocessing, run again in September 2026, gave:

- **nyl, dumped twice:** all 9,843 scans that the two runs share were identical point for point (one run had stopped early).
- **cp:** 5,252 scans instead of the paper's 5,259. All 5,250 scans in common were identical point for point.
- **garden, every scan:** all 273 of the paper's keyframe scans were present, and 244 of them were identical. Of the other 29, 28 differ by 1–41 points (most by 10 or fewer), out of about 4,500 per scan. One (folder `000235`) differs by 330 of its roughly 2,000 points.

The paper's cp and garden dumps were made earlier than the other four (garden in December 2025, cp in April 2026). garden's keyframes are also chosen by 4DRadarSLAM's own odometry, so a new garden dump can hold other scans than the paper's 273. The other five sequences keep every scan.

A new dump can therefore differ slightly from the paper's input, and the table of step 7 can then differ in the last digits. Step 5 remains the exact reference for Tables 1 and 3.

## 7. Rerun DRG on every scan pair (C)

Put the `dump:` paths in `config/datasets.yaml`. Check the setup on garden first (272 pairs, about 3 min), then run all six sequences:

```bash
python3 scripts/reproduce_paper.py --datasets config/datasets.yaml --sequences garden --out ~/drg_out/rerun
python3 scripts/reproduce_paper.py --datasets config/datasets.yaml --jobs 16 --out ~/drg_out/rerun   # --jobs: at most nproc
```

`reproduce_paper.py` prints one line per finished task and works in four parts:

1. It registers every consecutive pair of each dump in tasks of 200 pairs, with the paper's parameters and `config/norm_stats/<seq>.yaml`. Every pair is seeded, so the estimates do not depend on `--jobs`.
2. It writes `<out>/relative_poses/<seq>.txt`. For cp it also writes `cp_across_missing_scans.txt`, which holds the two pairs that span a scan without a point cloud.
3. It prints the table of step 5.
4. In the table's last column, it gives the largest difference from `results/relative_poses/`.

With the paper's 40 RANSAC trials, the mean time per pair and process in the release rerun was 0.8 s on loop3, 0.9 s on garden, 1.5 s on loop2, 1.6 s on loop1, 3.2 s on nyl and 3.7 s on cp. That is 29 CPU-hours for the six sequences, about 2 h with 16 processes.

- **On the paper's dumps,** the table must equal step 5 to the last digit, and the largest difference must be about 1e-7 or less. In a fresh environment it was `0.0e+00 over 272 pairs` for garden. The released estimates themselves agree with the paper's original run to within 2e-7.
- **On new dumps from step 6,** report the pair counts and every difference from step 5. Do not change any parameter to close a gap.
- Never pass `--out results` without `--score-only`, because that overwrites the released estimates.

## 8. Run the ROS node on a rosbag (D)

A 10 s test on loop3 (`play_args` is passed to `rosbag play`; leave it out to play the whole sequence):

```bash
roslaunch drg ntu4dradlm.launch sequence:=loop3 output_dir:=$HOME/drg_out/loop3 \
    bags:="$(ls <NTU4DRadLM>/loop3/*.bag | xargs)" rviz:=false ransac_trials:=1 play_args:="-s 60 -u 10"
```

- **Startup.** `rosbag play` first indexes the bags (about 1 min per 3 GB file) and waits for the node, so nothing happens for a while.
- **Progress.** The node logs `[DRG] scan N` for every scan. It never drops a scan; it only falls behind. When the last scan is logged, stop it with Ctrl-C (SIGINT, not `kill -9`), and it prints `[DRG] processed N scans`. The 10 s test registers 120 scans, so `relative_poses.txt` gets 119 lines.
- **Health checks** while it runs, from a second shell with the same ROS environment:

  | command | expected |
  |---|---|
  | `rostopic info /radar_enhanced_pcl` | `Subscribers:` lists `/drg_odometry` |
  | `rostopic hz /drg_odometry/odometry` | about 12 Hz in bag time while the node keeps up |
  | `rostopic echo -n1 /drg_odometry/odometry` | `frame_id: "odom"`, `child_frame_id: "radar_body"` |
  | `rosrun tf tf_echo odom radar_body` | the same pose as TF (the first lines can say that `odom` does not exist yet) |

- **Memory.** The node's input queue is unbounded, so that no scan is dropped. When playback outpaces the node, every waiting scan stays in memory: about 180 kB per message on the wire on loop3 (64–285 kB), and more once deserialised in Python. Keep the default `rate` of 0.05 for whole sequences.
- **Options.** `rviz:=false` is for a headless machine; leave it out to watch the trajectory in RViz. `ransac_trials:=1` is faster than the paper's 40 and gave the same estimates on all of the paper's windowed pairs. `rate` (default 0.05) is the playback rate.
- **Output.** `output_dir` gets `relative_poses.txt` and `stamp_est_traj.txt` (TUM), flushed after every pair. Score them with `evaluate_pairs.py` and `evaluate_trajectory.py` (step 5) or with evo (step 9).
- **Input.** This run registers the raw `/radar_enhanced_pcl` scans, not the paper's input, so the numbers come out close to Table 1 but not equal. On loop3, registering the raw scans offline gave a mean TE of 0.0776 m, against 0.0778 m on the paper's input. For the paper's input, run 4DRadarSLAM (Go-RIO `65cfefc`, step 6) on the same bag, and point the node at its output instead:

```bash
roslaunch drg drg.launch input_topic:=/filtered_points cloud_type:=PointCloud2 rotate_to_body:=false \
    norm_stats:=$(rospack find drg)/config/norm_stats/loop3.yaml output_dir:=$HOME/drg_out/loop3_paper
```

## 9. Optional: whole-run trajectories with evo

```bash
python3 scripts/export_trajectories.py --datasets config/datasets.yaml --est-dir results/relative_poses \
    --out-dir ~/drg_out/trajectories --segmented
cd ~/drg_out/trajectories/loop3
evo_ape tum stamp_gt_traj.txt stamp_est_traj.txt -va --save_results evo_ape.zip --save_plot evo_ape.pdf --plot_mode xy --no_warnings
evo_ape tum stamp_gt_traj.txt stamp_est_traj_segmented.txt -va --no_warnings
```

The exported files must equal `results/trajectories/<seq>/` byte for byte (check with `cmp`). Their ATE, the RMSE of `evo_ape -va` in metres:

| | cp | garden | nyl | loop1 | loop2 | loop3 |
|---|---|---|---|---|---|---|
| whole run | 1.61 | 1.02 | 9.66 | 538.74 | 121.86 | 58.38 |
| Fig. 4 version | 1.24 | 1.02 | 9.66 | 20.65 | 52.94 | 15.00 |

`garden_every_scan`, which is not in the paper, gives 3.15 m.

The two trajectory files:

- `stamp_est_traj.txt` chains every pair of the run into one trajectory.
- `stamp_est_traj_segmented.txt` is the version drawn in Fig. 4. As in the paper, a pair is scored only if both of its scans lie within 0.1 s of a ground-truth sample. The 10 Hz ground truth has short gaps, and the chain is restarted where a scan falls in one: 21 times on loop1, 9 on loop2 and 5 on loop3.

The paper reports no ATE, because over long chains it mainly reflects where the heading error accumulates. Instead, the paper scores the chained estimates by t_rel and r_rel (Table 3).

Things to know when running evo:

- `stamp_gt_traj.txt` is the ground truth interpolated at the radar stamps (`scripts/interpolate_gt.py`). Do the same for any new estimate rather than rely on evo's nearest-stamp matching. The radar runs at 12 Hz and the ground truth at 10 Hz, so the nearest sample can be 50 ms off.
- In scripts, always pass `--no_warnings`. Otherwise evo waits for keyboard input before it overwrites a saved result.
- `bash scripts/evo_ape_all.sh` reruns every `evo_ape` of `results/` in place. The RMSE values must not change. Other evo versions print different log lines, so the rewritten files show up in `git status`. Restore them with `git checkout -- results`.

## 10. Optional: beyond the paper

- **DRG as the front end of 4DRadarSLAM, without and with loop closure.** See `results/slam_backend/README.md`, section Reproduce.
  - Re-scoring the released runs takes seconds: `bash scripts/evo_ape_all.sh slam_backend`.
  - Running the system again needs ROS Noetic, Go-RIO and the bags.
  - With loop closure, the back end gives different results on identical input: loop2 gave 14.09, 57.26 and 101.11 m in 3 runs, and loop3 gave 11.85–21.63 m in 7. Report every run, not a single number.
- **SNAIL-Radar.** See `results/snail_radar/README.md`. Its ground truth is not included (ODbL licence): the user downloads it, and `scripts/snail_radar/` converts it.

## 11. Report

Tell the user which level was reached, and show each table or number next to its expected value above. List every difference, even in the last digit, with its cause if known. Do not change the parameters, the Open3D version or the scoring to make a number match.

## Conventions

- **Relative poses.** `relative_poses.txt` has one line per pair: `stamp_target stamp_source`, then the top 3×4 of **T**, row by row. **T** maps points of the source scan (the later one) into the frame of the target scan.
- **Frame.** All poses are in the radar-centred body frame. Its origin is at the radar, and its axes are those of the Livox lidar and the ground truth: x forward, y left, z up. The raw NTU4DRadLM radar points are rotated into it by 3.13° (`rotate_to_body`); the radar-to-lidar translation is not applied.
- **Errors.** With T_err = T_gt T_est⁻¹, TE = |t_err| in metres and RE = arccos((tr R_err − 1) / 2) in degrees. The ground truth is interpolated at the radar stamps (linear in position, SLERP in rotation).
- **Scans.** A scan is an (N, 5) array: x, y, z [m], RCS [dB] and Doppler [m/s]. Doppler is the raw signed range rate, not ego-compensated, so static points the radar approaches are negative. DRG fits d = −n·v.
- **Stamps.** `sec + nsec·1e-9`, from the `data` file of each dump folder, or from the message header in the node.
- **Radar timestamps.** On NTU4DRadLM, the intervals between radar stamps alternate long and short. The ground truth is interpolated at these stamps, and the Doppler translation uses the same Δt, which can favour DRG in per-pair TE (paper, Sec. 4.1). Mention this when comparing DRG with other methods.

## Running DRG on another radar

[docs/PARAMETERS.md](docs/PARAMETERS.md#adapting-drg-to-another-radar) lists the parameters to revisit. Before trusting any number on a new radar, also check the input itself. Each of these checks mattered on SNAIL-Radar (`results/snail_radar/README.md`):

- **Doppler sign.** For a platform driving forward, `DRG(DRGParams(), stats).prepare(scan).velocity` must point along +x at about the platform's speed. That is the Soft-REVE ego-velocity in m/s, in the radar frame. If it points along −x, the Doppler sign is flipped.
- **Frame.** Points must be in a forward-left-up frame centred on the radar. In the node, set `rotate_to_body:=false` for anything other than NTU4DRadLM's raw radar frame.
- **RCS units.** DRG drops returns below `min_rcs` = 0 dB. A radar that reports RCS in dBsm, mostly below 0 (the Continental ARS548), needs that floor switched off: pass `--params config/snail_radar/ars548.yaml` to `register_dump.py`.
- **Normalisation statistics.** Compute them for every sequence with `scripts/compute_norm_stats.py`, passing the same `--params` as the run.
- **Gate thresholds.** `tau_rcs` (1 dB) and `tau_residual` (0.01 m/s) were set on the Oculii Eagle, whose RCS field is an SNR in 0.01 dB steps and whose Doppler residuals are tight. Measure the new radar first with `scripts/gate_repeatability.py --dump <dump> --gt <gt.txt> [--params ...]`: it reports how many true matches (ground-truth motion, 0.1 m) pass each test. On the paper's data the gate keeps 88 % of them; on the ARS548, which reports an integer dBsm and a noisier Doppler, the paper's values keep 34 %, and the paper's own rules (`tau_residual` at 3–10× the median residual difference of true matches, `tau_rcs` wide enough for the RCS quantisation) give 2 dB and 0.05 m/s (`config/snail_radar/ars548_gate.yaml`, 83 % kept). Report runs with the paper's thresholds and with the re-tuned ones side by side, as `results/snail_radar/` does.
- **Stamps.** They must strictly increase. Two scans with the same stamp give Δt = 0, and `fusion_weight` then divides by zero. SNAIL's ARS548 `times.txt` lists 6 stamps twice, and `scripts/snail_radar/snail_to_dump.py` keeps one scan per stamp. The ROS node skips a scan whose stamp does not increase.
- **Accumulated clouds.** Check that the geometry moves with the stamps. SNAIL's Oculii accumulative cloud advances by one 15 Hz frame of motion per scan, whatever the time between stamps, so scan matching on it under-measures the motion, rotation included.
- **RANSAC trials.** On these points, 1 trial gave the same estimates as 40:
  - the paper's NTU4DRadLM windowed pairs;
  - SNAIL's dense Oculii accumulative clouds.

  On sparse clouds (SNAIL's single-frame Oculii cloud and the ARS548), the released runs keep 40.

## Troubleshooting

- **`No matching distribution found for open3d==0.18.0`.** Python is 3.12 or newer, which has no wheel, or pip is older than 20.3 and needs an upgrade.
- **The window check fails, or estimates differ by more than about 1e-9.** Check that `open3d.__version__` is 0.18.0. The scripts set `OMP_NUM_THREADS=1` before importing numpy and Open3D, because more threads change the last digits (about 1e-11 m). Set it too when you call the library from your own code.
- **nyl gives 11,231 pairs and t_rel 6.49 %.** `--max-gt-gap 0.15` is missing.
- **`FileNotFoundError: no frame folders with cloud.pcd`.** The `dump:` path must be the folder that directly holds the `%06d/` folders.
- **Go-RIO prints `failed to allocate solver`, then the nodelet manager dies.** The g2o build has no CHOLMOD, but the launch files ask for `lm_var_cholmod`. Set `g2o_solver_type` to `lm_var_csparse`, as the paper's runs did. That changes only the pose graph; the dumped clouds come before it.
- **A dump has far fewer folders than the table in step 6.** Either a node crashed, or nothing published `/odom`. 4DRadarSLAM creates keyframes only from scans that have scan-matching odometry. Check the launch log.
- **The ROS node falls far behind the bag.** This is expected for a Python implementation: about 1–4 s per pair with 40 RANSAC trials (measured on the paper's input, step 7), against 83 ms between scans. The 10 s test with `ransac_trials:=1` took 0.05–0.15 s per pair. Use `ransac_trials:=1` or a lower `rate`; the node never drops a scan.
- **Another radar.** See "Running DRG on another radar" above.
