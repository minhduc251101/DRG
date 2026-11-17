"""Register consecutive pairs (k, k+1) of a dump, reusing each prepared scan once."""
from __future__ import annotations

import time
from typing import Iterator, List, Optional, Tuple

from .evaluation import GroundTruth, pose_error
from .io import DumpSequence
from .registration import DRG


def register_pairs(drg: DRG, seq: DumpSequence, pairs: List[Tuple[int, int]],
                   gt: Optional[GroundTruth] = None) -> Iterator[dict]:
    """Yield one record per pair (target k, source k+1); TE / RE are added when GT is available."""
    previous = (None, None)              # the source of one pair is the target of the next
    for k_t, k_s in pairs:
        if k_t not in seq or k_s not in seq:
            continue
        t_t, t_s = seq.stamp(k_t), seq.stamp(k_s)
        tic = time.time()
        target = previous[1] if previous[0] == k_t else drg.prepare(seq.points(k_t))
        source = drg.prepare(seq.points(k_s))
        previous = (k_s, source)
        res = drg.register(source, target, dt=t_s - t_t)

        row = {"target": k_t, "source": k_s, "stamp_target": t_t, "stamp_source": t_s,
               "dt": t_s - t_t, "T": res.T, "success": res.success,
               "fusion_weight": res.fusion_weight, "n_putative": res.n_putative,
               "n_gated": res.n_gated, "n_inliers": res.n_inliers, "time_s": time.time() - tic}
        T_gt = gt.relative(t_t, t_s) if gt is not None else None
        if T_gt is not None:
            row["te"], row["re"] = pose_error(res.T, T_gt)
        yield row
