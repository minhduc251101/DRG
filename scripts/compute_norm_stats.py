#!/usr/bin/env python3
"""Compute the z-score statistics of the four radar channels for one sequence.

As in the paper, the statistics are taken over every 10th scan of the sequence being evaluated.

    python scripts/compute_norm_stats.py --dump <dump_root> --out config/norm_stats/cp.yaml
"""
import argparse
import os
import sys

# Single-threaded, as for the paper: with more threads (BLAS / OpenMP) the estimates change in
# the last digits (~1e-11 m). This must run before numpy and open3d are imported.
os.environ["OMP_NUM_THREADS"] = "1"

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from drg import DRGParams, compute_norm_stats  # noqa: E402
from drg.io import DumpSequence  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True, help="dump root with %%06d/cloud.pcd frame folders")
    ap.add_argument("--out", required=True, help="output YAML")
    ap.add_argument("--every", type=int, default=10, help="use every n-th scan (paper: 10)")
    ap.add_argument("--params", default=None,
                    help="DRG parameter YAML, if the run uses non-default limits (e.g. config/snail_radar/ars548.yaml)")
    args = ap.parse_args()

    seq = DumpSequence(args.dump)
    indices = seq.indices()[::args.every]
    print(f"{len(indices)} of {len(seq.indices())} scans")
    params = DRGParams.from_yaml(args.params) if args.params else DRGParams()
    stats = compute_norm_stats((seq.points(k) for k in indices), params)
    name = os.path.basename(os.path.normpath(args.dump))
    stats.to_yaml(args.out, comment=f"Radar-channel statistics of {name}: "
                                    f"every {args.every}th of {len(seq.indices())} scans.")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
