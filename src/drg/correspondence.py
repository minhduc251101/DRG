"""Stages 3-4 - mutual nearest neighbours in descriptor space and the Doppler-RCS gate (Sec. 3.4)."""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree

from .params import DRGParams


def mutual_nearest_neighbours(source_desc: np.ndarray, target_desc: np.ndarray):
    """Pairs (i, j) where j is the nearest target of i AND i is the nearest source of j."""
    source_to_target = cKDTree(target_desc).query(source_desc)[1]
    target_to_source = cKDTree(source_desc).query(target_desc)[1]
    i = np.arange(len(source_desc))
    mutual = target_to_source[source_to_target] == i
    return i[mutual], source_to_target[mutual]


def doppler_rcs_gate(i, j, source, target, p: DRGParams):
    """Keep a correspondence only if both returns agree in RCS and in Doppler residual (Eq. 4),
    and weight it by its consistency (Eq. 5):

        w_ij = sqrt(w_i w_j) * exp(-drho^2 / 2 tau_rho^2) * exp(-de^2 / 2 tau_e^2)

    Args:
        i, j: source / target indices of the putative correspondences.
        source, target: PreparedScan (fields rcs, residual, weight).
    Returns:
        i, j, w_ij of the kept correspondences.
    """
    d_rcs = np.abs(source.rcs[i] - target.rcs[j])
    d_res = np.abs(source.residual[i] - target.residual[j])
    keep = (d_rcs <= p.tau_rcs) & (d_res <= p.tau_residual)
    weight = (np.sqrt(source.weight[i] * target.weight[j])
              * np.exp(-d_rcs**2 / (2 * p.tau_rcs**2))
              * np.exp(-d_res**2 / (2 * p.tau_residual**2)))
    return i[keep], j[keep], weight[keep]
