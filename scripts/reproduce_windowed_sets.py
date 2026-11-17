#!/usr/bin/env python3
"""Reproduce the windowed-set results of the paper and check them against the released estimates.

Runs DRG on the 37 windowed sets (30 consecutive pairs each) listed in
tests/reference/windowed_sets_seed42.json, compares every estimated transform with the one
produced for the paper, and prints the pooled TE / RE of the windowed sets per sequence.

    python scripts/reproduce_windowed_sets.py --datasets config/datasets.yaml [--sets cp_set1_4470 ...]

config/datasets.yaml maps each sequence to its dump, ground truth and GT gap limit (see the
example file; relative paths are relative to that file). example/datasets.yaml points at the
window shipped in example/data, so one set can be checked without the dataset:

    python scripts/reproduce_windowed_sets.py --datasets example/datasets.yaml --sets loop3_set4_5692
 With Open3D 0.18.0 the estimates match the paper's up to floating-point rounding
(most pairs bit-identical, all within 1e-9 m). Other Open3D versions were not tested.
"""
import argparse
import json
import os
import sys
from collections import defaultdict

# Single-threaded, as for the paper: with more threads (BLAS / OpenMP) the estimates change in
# the last digits (~1e-11 m). This must run before numpy and open3d are imported.
os.environ["OMP_NUM_THREADS"] = "1"

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from drg import DRG, DRGParams, NormStats  # noqa: E402
from drg.evaluation import GroundTruth  # noqa: E402
from drg.io import DumpSequence  # noqa: E402
from drg.sequence import register_pairs  # noqa: E402

REFERENCE = os.path.join(HERE, "..", "tests", "reference", "windowed_sets_seed42.json")
STATS_DIR = os.path.join(HERE, "..", "config", "norm_stats")
EXCLUDED = {("garden", 254)}   # 5.9 s sensor dropout, excluded from the paper's windowed tables


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--datasets", required=True)
    ap.add_argument("--sets", nargs="*", default=None, help="subset of set tags (default: all 37)")
    ap.add_argument("--tolerance", type=float, default=1e-9, help="max |T - T_paper| to pass")
    args = ap.parse_args()

    with open(args.datasets) as f:
        datasets = yaml.safe_load(f)
    base = os.path.dirname(os.path.abspath(args.datasets))
    with open(REFERENCE) as f:
        reference = json.load(f)["sets"]
    tags = args.sets or list(reference)

    drgs, sequences, gts = {}, {}, {}
    errors, worst, n_identical, n_pairs = defaultdict(list), 0.0, 0, 0
    for tag in tags:
        ref = reference[tag]
        name = ref["sequence"]
        if name not in drgs:
            cfg = datasets[name]
            drgs[name] = DRG(DRGParams(), NormStats.from_yaml(os.path.join(STATS_DIR, f"{name}.yaml")))
            sequences[name] = DumpSequence(os.path.join(base, cfg["dump"]))
            gts[name] = GroundTruth(os.path.join(base, cfg["gt"]), cfg.get("max_gt_gap", 0.1))
        expected = {p["target"]: p for p in ref["pairs"]}
        pairs = [(ref["start"] + n, ref["start"] + n + 1) for n in range(ref["num_pairs"])]

        for row in register_pairs(drgs[name], sequences[name], pairs, gts[name]):
            if row["target"] not in expected:
                continue                                  # pair without GT in the paper run
            diff = float(np.abs(row["T"] - np.array(expected[row["target"]]["T"])).max())
            worst, n_pairs = max(worst, diff), n_pairs + 1
            n_identical += diff == 0.0
            if (name, row["target"]) not in EXCLUDED and "te" in row:
                errors[name].append((row["te"], row["re"]))
        print(f"{tag:18s} done  (worst |dT| so far {worst:.1e})", flush=True)

    print(f"\n{n_pairs} pairs compared with the paper: {n_identical} bit-identical, max |dT| = {worst:.2e}")
    print(f"\n{'sequence':10s} {'pairs':>6s} {'TE mean [m]':>12s} {'RE mean [deg]':>14s}")
    for name, e in errors.items():
        e = np.array(e)
        print(f"{name:10s} {len(e):6d} {e[:, 0].mean():12.4f} {e[:, 1].mean():14.3f}")
    if worst > args.tolerance:
        sys.exit(f"\nFAILED: max |dT| {worst:.2e} > tolerance {args.tolerance:.0e}")
    print("\nPASSED")


if __name__ == "__main__":
    main()
