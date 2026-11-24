"""Readers for the per-scan dumps of 4DRadarSLAM / Go-RIO used in the paper.

A dump is a folder of frame folders `%06d/` with
    cloud.pcd   binary PCD, 8 float32 fields: x y z intensity normal_x normal_y normal_z curvature
                (intensity = RCS [dB], curvature = Doppler [m/s]; some dumps name them rcs/doppler)
    data        text; the line `stamp <sec> <nsec>` is the radar timestamp
"""
from __future__ import annotations

import os
import re
from typing import Dict

import numpy as np

RCS_FIELDS = ("rcs", "intensity", "Power")
DOPPLER_FIELDS = ("doppler", "curvature", "Doppler")


def read_pcd(path: str) -> np.ndarray:
    """Binary PCD -> (N, 5) float32 array x, y, z, RCS, Doppler."""
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii").strip()
            key, *values = line.split()
            header[key] = values
            if key == "DATA":
                break
        if header["DATA"][0] != "binary":
            raise ValueError(f"{path}: only binary PCD is supported")
        names = header["FIELDS"]
        n = int(header["POINTS"][0])
        dtype = np.dtype([(name, np.float32) for name in names])
        raw = np.frombuffer(f.read(n * dtype.itemsize), dtype=dtype)
    rcs = next(name for name in names if name in RCS_FIELDS)
    doppler = next(name for name in names if name in DOPPLER_FIELDS)
    return np.column_stack([raw["x"], raw["y"], raw["z"], raw[rcs], raw[doppler]])


def stamp_to_sec(sec: int, nsec: int) -> float:
    """ROS stamp -> seconds, computed exactly as for the paper (sec + nsec * 1e-9)."""
    return sec + nsec * 1e-9


def read_stamp(path: str) -> float:
    with open(path) as f:
        m = re.search(r"stamp\D+(\d{10})\D+(\d{1,9})", f.read())
    if m is None:
        raise ValueError(f"no 'stamp <sec> <nsec>' line in {path}")
    return stamp_to_sec(int(m.group(1)), int(m.group(2)))


class DumpSequence:
    """Frames of one dump, addressed by their folder number (not by list position)."""

    def __init__(self, root: str):
        self.root = root
        self.frames: Dict[int, str] = {
            int(name): os.path.join(root, name)
            for name in os.listdir(root)
            if name.isdigit() and os.path.isfile(os.path.join(root, name, "cloud.pcd"))
        }
        if not self.frames:
            raise FileNotFoundError(f"no frame folders with cloud.pcd under {root}")

    @classmethod
    def from_frames(cls, root: str, frames: Dict[int, str]) -> "DumpSequence":
        """A dump whose frame folders are already known: {index: folder}. Skips listing the dump."""
        seq = cls.__new__(cls)
        seq.root, seq.frames = root, dict(frames)
        return seq

    def indices(self):
        return sorted(self.frames)

    def __contains__(self, index: int) -> bool:
        return index in self.frames

    def points(self, index: int) -> np.ndarray:
        return read_pcd(os.path.join(self.frames[index], "cloud.pcd"))

    def stamp(self, index: int) -> float:
        return read_stamp(os.path.join(self.frames[index], "data"))
