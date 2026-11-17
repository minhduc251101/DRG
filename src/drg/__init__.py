"""DRG - Doppler-RCS Gated registration of consecutive 4D radar scans (ACRA 2026)."""
import os

# Every number in the paper was computed single-threaded; with more threads (BLAS / OpenMP) the
# estimates change in the last digits. Effective only if drg is imported before numpy / open3d,
# so the scripts and the ROS node also set it at the very top.
os.environ["OMP_NUM_THREADS"] = "1"

from .descriptor import NormStats, compute_norm_stats  # noqa: E402
from .params import DRGParams  # noqa: E402
from .registration import DRG, PreparedScan, RegistrationResult  # noqa: E402

__all__ = ["DRG", "DRGParams", "NormStats", "PreparedScan", "RegistrationResult",
           "compute_norm_stats"]
__version__ = "1.0.0"
