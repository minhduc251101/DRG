#!/usr/bin/env python3
"""Reproduce the paper's DRG results by running DRG on every consecutive scan pair (Tables 1 and 3).

    python3 scripts/reproduce_paper.py --datasets config/datasets.yaml --jobs 16 --out rerun

For each sequence (default: the six of the paper) it
  1. registers every consecutive scan pair (k, k+1) of the sequence's dump with DRG, the paper's
     parameters and config/norm_stats/<seq>.yaml, in --jobs parallel processes. Every pair is seeded, so
     the estimates do not depend on the number of processes or on how the pairs are split;
  2. writes <out>/relative_poses/<seq>.txt, in the format of results/relative_poses/. Where a scan of the
     dump has no point cloud (cp: frames 3247 and 3312) the pairs touching it do not exist; the pair
     that spans it is registered too and written to <out>/relative_poses/<seq>_across_missing_scans.txt,
     for the whole-run trajectory only;
  3. scores the estimates exactly as the paper: TE and RE of every pair whose two frames have ground
     truth (Table 1) and the KITTI drift t_rel / r_rel of the chained estimates (Table 3);
  4. reports the largest difference to the released estimates of results/relative_poses/.

Without dumps, the released estimates can be scored the same way (seconds; needs only the ground truth):

    python3 scripts/reproduce_paper.py --datasets config/datasets.yaml --score-only --out results

The trajectories (whole run, and the version of Fig. 4) and their evo_ape -va results then follow with

    python3 scripts/export_trajectories.py --datasets config/datasets.yaml --est-dir rerun/relative_poses \\
        --out-dir rerun/trajectories --segmented
    cd rerun/trajectories/loop1 && evo_ape tum stamp_gt_traj.txt stamp_est_traj.txt -va --save_results evo_ape.zip --save_plot evo_ape.pdf

DRG takes about 1-5 s per pair and process: all six sequences (59,621 pairs) take about 2 h with 16
processes. config/datasets.yaml gives the dump, the ground truth and its gap limit of each sequence (see
config/datasets_example.yaml); --sequences selects some of them, e.g. --sequences loop1 loop2.
"""
import argparse
import os
import sys
import time

# Single-threaded per process, as for the paper: with more threads (BLAS / OpenMP) the estimates change
# in the last digits (~1e-11 m). This must run before numpy and open3d are imported.
os.environ["OMP_NUM_THREADS"] = "1"
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(REPO, "src"))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from drg.evaluation import GroundTruth, relative_pose_line, score_relative_poses  # noqa: E402

SEQUENCES = ("cp", "garden", "nyl", "loop1", "loop2", "loop3")
CHUNK = 200                     # pairs per task
_WORKER = {}                    # per process: the DRG instance and the dump of the current sequence


def register_task(task):
    """Worker: register a list of pairs (k_t, k_s) of one dump. Returns (name, first target, lines)."""
    from drg import DRG, DRGParams, NormStats
    from drg.io import DumpSequence
    from drg.sequence import register_pairs

    name, dump, stats, pairs, frames = task
    if _WORKER.get("stats") != stats:
        _WORKER.update(stats=stats, drg=DRG(DRGParams(), NormStats.from_yaml(stats)))
    seq = DumpSequence.from_frames(dump, frames)        # the frames of this task, listed once by the main process
    lines = [relative_pose_line(r["stamp_target"], r["stamp_source"], r["T"])
             for r in register_pairs(_WORKER["drg"], seq, pairs)]
    return name, pairs[0][0], lines


def write(path, header, lines):
    with open(path, "w") as f:
        f.writelines(f"# {h}\n" for h in header)
        f.write("# stamp_target stamp_source then T (source -> target frame), top 3x4 row-major\n")
        f.writelines(lines)


def max_difference(path_a, path_b):
    """Largest difference of any matrix entry over the pairs (matched by stamps) in both files."""
    a, b = np.loadtxt(path_a, ndmin=2), np.loadtxt(path_b, ndmin=2)
    index = {(round(r[0], 6), round(r[1], 6)): r for r in b}
    common = [(r, index[(round(r[0], 6), round(r[1], 6))]) for r in a if (round(r[0], 6), round(r[1], 6)) in index]
    if not common:
        return 0, None
    return len(common), max(float(np.abs(x[2:14] - y[2:14]).max()) for x, y in common)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--datasets", required=True, help="YAML with dump, gt and max_gt_gap per sequence")
    ap.add_argument("--sequences", nargs="+", default=None, help="default: the six sequences of the paper")
    ap.add_argument("--jobs", type=int, default=os.cpu_count(), help="parallel processes (default: all cores)")
    ap.add_argument("--out", default="rerun", help="output folder (default: rerun)")
    ap.add_argument("--score-only", action="store_true",
                    help="skip the registration and score <out>/relative_poses/ as it is; needs only the ground "
                         "truth, e.g. --out results scores the released estimates")
    args = ap.parse_args()

    from drg.io import DumpSequence

    with open(args.datasets) as f:
        datasets = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(args.datasets))
    names = args.sequences or [s for s in SEQUENCES if s in datasets]
    out_dir = os.path.join(args.out, "relative_poses")
    os.makedirs(out_dir, exist_ok=True)

    if args.score_only:
        score_table(names, datasets, base, out_dir)
        return

    tasks, info = [], {}
    for name in names:
        cfg = datasets[name]
        dump = os.path.join(base, cfg["dump"])
        stats = os.path.join(base, cfg["stats"]) if "stats" in cfg else \
            os.path.join(REPO, "config", "norm_stats", f"{name}.yaml")
        sequence = DumpSequence(dump)                      # folders with a point cloud, listed once
        frames = sequence.indices()

        def task(task_name, pairs):
            needed = {k for pair in pairs for k in pair if k in sequence}
            return (task_name, dump, stats, pairs, {k: sequence.frames[k] for k in needed})

        consecutive = [(k, k + 1) for k in range(frames[0], frames[-1])]
        across = [(a, b) for a, b in zip(frames[:-1], frames[1:]) if b != a + 1]
        tasks += [task(name, consecutive[i:i + CHUNK]) for i in range(0, len(consecutive), CHUNK)]
        tasks += [task(f"{name}_across_missing_scans", [pair]) for pair in across]
        info[name] = {"frames": len(frames), "across": across, "cfg": cfg}
        print(f"{name}: {len(frames)} scans in {dump}, stats {os.path.relpath(stats)}", flush=True)

    print(f"registering {sum(len(t[3]) for t in tasks)} pairs in {len(tasks)} tasks with {args.jobs} processes",
          flush=True)
    results, t0 = {}, time.time()
    if args.jobs > 1:
        import multiprocessing as mp
        with mp.get_context("spawn").Pool(args.jobs) as pool:
            for n, (name, first, lines) in enumerate(pool.imap_unordered(register_task, tasks), 1):
                results.setdefault(name, {})[first] = lines
                print(f"  [{n}/{len(tasks)}] {name} from frame {first}: {len(lines)} pairs "
                      f"({time.time() - t0:.0f} s)", flush=True)
    else:
        for n, task in enumerate(tasks, 1):
            name, first, lines = register_task(task)
            results.setdefault(name, {})[first] = lines
            print(f"  [{n}/{len(tasks)}] {name} from frame {first}: {len(lines)} pairs ({time.time() - t0:.0f} s)",
                  flush=True)

    for name in names:
        lines = [l for first in sorted(results.get(name, {})) for l in results[name][first]]
        path = os.path.join(out_dir, f"{name}.txt")
        write(path, [f"DRG per-pair estimates on {name}: every consecutive scan pair of the dump ({len(lines)} pairs)",
                     "written by scripts/reproduce_paper.py"], lines)
        if info[name]["across"]:
            extra = [l for first in sorted(results[f"{name}_across_missing_scans"])
                     for l in results[f"{name}_across_missing_scans"][first]]
            write(os.path.join(out_dir, f"{name}_across_missing_scans.txt"),
                  [f"DRG estimates of the pairs that span the scans of the {name} dump without a point cloud",
                   "used only to chain the whole-run trajectory; the paper does not score them"], extra)
    score_table(names, datasets, base, out_dir)


def score_table(names, datasets, base, out_dir):
    """Score <out_dir>/<seq>.txt as in the paper and print Tables 1 and 3, with the difference to the release."""
    rows = []
    for name in names:
        cfg = datasets[name]
        path = os.path.join(out_dir, f"{name}.txt")
        score = score_relative_poses(path, GroundTruth(os.path.join(base, cfg["gt"]), cfg.get("max_gt_gap", 0.1)))
        released = os.path.join(REPO, "results", "relative_poses", f"{name}.txt")
        same = os.path.exists(released) and os.path.samefile(path, released)
        n_common, diff = max_difference(path, released) if os.path.exists(released) and not same else (0, None)
        n_pairs = sum(1 for line in open(path) if not line.startswith("#"))
        rows.append((name, n_pairs, score, n_common, diff, same))

    print(f"\n{'sequence':10s}{'pairs':>7s}{'scored':>8s}{'mean TE':>9s}{'med TE':>8s}{'mean RE':>9s}{'med RE':>8s}"
          f"{'t_rel %':>9s}{'r_rel':>8s}   largest difference to results/relative_poses")
    for name, n, s, n_common, diff, same in rows:
        drift = (f"{s['t_rel']:9.2f}{s['r_rel']:8.4f}" if s["t_rel"] is not None else f"{'-':>9s}{'-':>8s}")
        cmp = ("(the released file itself)" if same else
               f"{diff:.1e} over {n_common} pairs" if diff is not None else "(no released file)")
        print(f"{name:10s}{n:7d}{len(s['te']):8d}{s['te'].mean():9.4f}{np.median(s['te']):8.4f}"
              f"{s['re'].mean():9.3f}{np.median(s['re']):8.3f}{drift}   {cmp}")
    print("\nTE [m] and RE [deg] per pair (Table 1); t_rel [%] and r_rel [deg/m] over 100-800 m (Table 3).")


if __name__ == "__main__":
    main()
