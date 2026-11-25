#!/usr/bin/env python3
"""How repeatable are RCS and the ego-compensated Doppler residual between consecutive scans of a radar, and what
does DRG's Doppler-RCS gate do to the true matches? Use it before running DRG on a new radar (AGENTS.md, "Running DRG
on another radar"): the gate thresholds tau_rcs and tau_residual were set on the Oculii Eagle of NTU4DRadLM.

Protocol of the paper (Sec. 4.2): a return of the source scan that the ground-truth motion maps within --radius of a
return of the target scan is a true match. For these the script reports the fraction with identical RCS, the median
|dRCS| and |de| (e_i = d_i + n_i.v with Soft-REVE's velocity of each scan, stage 1), the fraction that passes each
gate test and both, and the same for randomly paired returns. The last block scans a few threshold pairs.

  python3 scripts/gate_repeatability.py --dump <dump> --gt <gt_odom.txt> [--params config/snail_radar/ars548.yaml]
                                        [--pairs 150] [--max-gt-gap 0.1] [--radius 0.1]

The paper's rule for a new radar: tau_residual at 3 to 10 times the median |de| of true matches (33x on cp), tau_rcs
wide enough that most true matches pass the RCS test alone (the ARS548 quantises RCS to 1 dBsm; 2 dB keeps 88 %).
"""
import argparse
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from drg import DRGParams  # noqa: E402
from drg.ego_velocity import doppler_residual, field_of_view_mask, soft_reve  # noqa: E402
from drg.evaluation import GroundTruth  # noqa: E402
from drg.io import DumpSequence  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dump", required=True, help="dump folder (%06d/cloud.pcd + data)")
    ap.add_argument("--gt", required=True, help="ground truth, TUM format, in the radar frame")
    ap.add_argument("--params", default=None, help="DRG parameter YAML (default: the paper's)")
    ap.add_argument("--pairs", type=int, default=150, help="evenly spaced consecutive pairs to test")
    ap.add_argument("--max-gt-gap", type=float, default=0.1, help="s, as in the evaluation scripts")
    ap.add_argument("--radius", type=float, default=0.1, help="m, true-match distance after the ground-truth motion")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--json", default=None, help="also write the statistics to this JSON file")
    a = ap.parse_args()
    p = DRGParams.from_yaml(a.params) if a.params else DRGParams()
    seq, gt = DumpSequence(a.dump), GroundTruth(a.gt, a.max_gt_gap)
    idx = seq.indices()
    rng = np.random.default_rng(a.seed)
    d_rcs, d_e, r_rcs, r_e, used = [], [], [], [], 0
    for k in [idx[i] for i in np.linspace(0, len(idx) - 2, min(a.pairs, len(idx) - 1)).astype(int)]:
        if k + 1 not in seq.frames:
            continue
        T = gt.relative(seq.stamp(k), seq.stamp(k + 1))  # maps source (k+1) points into the target (k) frame
        if T is None:
            continue
        tgt, src = seq.points(k), seq.points(k + 1)
        tgt, src = tgt[field_of_view_mask(tgt, p)], src[field_of_view_mask(src, p)]
        if len(tgt) < 10 or len(src) < 10:
            continue
        _, vt, _ = soft_reve(tgt, p, p.seed)
        _, vs, _ = soft_reve(src, p, p.seed)
        et, es = doppler_residual(tgt, vt), doppler_residual(src, vs)
        ps = (T[:3, :3] @ src[:, :3].T).T + T[:3, 3]
        dist, j = cKDTree(tgt[:, :3]).query(ps, distance_upper_bound=a.radius)
        ok = np.isfinite(dist)
        if ok.sum() == 0:
            continue
        jr = rng.integers(0, len(tgt), ok.sum())
        d_rcs.append(np.abs(src[ok, 3] - tgt[j[ok], 3])); d_e.append(np.abs(es[ok] - et[j[ok]]))
        r_rcs.append(np.abs(src[ok, 3] - tgt[jr, 3])); r_e.append(np.abs(es[ok] - et[jr]))
        used += 1
    if not used:
        sys.exit("no pair with ground truth and enough returns")
    dr, de, rr, re = map(np.concatenate, (d_rcs, d_e, r_rcs, r_e))
    print(f"{used} consecutive pairs, {len(dr)} true matches ({len(dr) / used:.0f} per pair), gate tau_rcs {p.tau_rcs} dB, tau_residual {p.tau_residual} m/s")
    print(f"  identical RCS {np.mean(dr == 0) * 100:.1f} %   median |dRCS| {np.median(dr):.2f} dB   median |de| {np.median(de):.4f} m/s   p90 |de| {np.percentile(de, 90):.4f} m/s")
    print(f"  true matches kept: RCS test {np.mean(dr <= p.tau_rcs) * 100:.1f} %, residual test {np.mean(de <= p.tau_residual) * 100:.1f} %, "
          f"both {np.mean((dr <= p.tau_rcs) & (de <= p.tau_residual)) * 100:.1f} %   (random pairs: both {np.mean((rr <= p.tau_rcs) & (re <= p.tau_residual)) * 100:.1f} %)")
    print("  other thresholds, true matches kept / random pairs kept:")
    grid = {}
    for te in (0.01, 0.02, 0.05, 0.1):
        for tr in (1.0, 2.0, 3.0):
            grid[f"{tr:g}dB_{te:g}mps"] = {"true": float(np.mean((dr <= tr) & (de <= te))), "random": float(np.mean((rr <= tr) & (re <= te)))}
        print("   " + "  ".join(f"{tr:.0f} dB & {te:.2f} m/s: {100 * grid[f'{tr:g}dB_{te:g}mps']['true']:5.1f} / {100 * grid[f'{tr:g}dB_{te:g}mps']['random']:4.1f} %"
                               for tr in (1.0, 2.0, 3.0)))
    if a.json:
        import json
        json.dump({"dump": a.dump, "gt": a.gt, "params": a.params, "pairs": used, "true_matches": int(len(dr)),
                   "tau_rcs": p.tau_rcs, "tau_residual": p.tau_residual,
                   "identical_rcs": float(np.mean(dr == 0)), "median_abs_drcs": float(np.median(dr)),
                   "median_abs_de": float(np.median(de)), "p90_abs_de": float(np.percentile(de, 90)),
                   "kept_rcs_test": float(np.mean(dr <= p.tau_rcs)), "kept_residual_test": float(np.mean(de <= p.tau_residual)),
                   "kept_both": float(np.mean((dr <= p.tau_rcs) & (de <= p.tau_residual))),
                   "random_kept_both": float(np.mean((rr <= p.tau_rcs) & (re <= p.tau_residual))), "grid": grid},
                  open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
