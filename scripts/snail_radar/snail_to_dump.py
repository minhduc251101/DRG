#!/usr/bin/env python3
"""Convert one SNAIL-Radar sequence folder into the dump layout read by Pipeline 6 (DRG):
<out>/%06d/{cloud.pcd,data}

  --topic enhanced : eagleg7/enhanced  (/radar_enhanced_pcl2, Oculii Eagle, accumulative)  RCS field 'Power'
  --topic pcl      : eagleg7/pcl       (/radar_pcl2, Oculii Eagle, single frame)           RCS field 'Power'
  --topic ars548   : ars548/points     (/ars548, Continental ARS548, single frame, 20 Hz)  RCS field 'intensity' [dBsm]

  cloud.pcd : binary PCD, float32 fields  x y z Power Doppler   (Power = RCS [dB], Doppler = raw range rate [m/s])
  data      : 'stamp <sec> <nsec>' (header stamp of the radar message, exact digits of the file name)

Points stay in the sensor frame. Without --reve-filter nothing is removed (DRG applies its own range / RCS / FoV
limits). With --reve-filter each scan is reduced to its REVE inliers (static returns): 3-point RANSAC + LS ego-velocity
on the Doppler (Doer & Trommer; the same estimator and parameters as DRG stage 1), keep |d_i + n_i.v| < 0.15 m/s.
--min-rcs sets the RCS floor of the field-of-view mask used by that filter (default: DRG's 0 dB).

  python snail_to_dump.py --seq data/20240113_1 --topic ars548 --reve-filter --min-rcs -1000 --out dumps_ars548/20240113_1
"""
import argparse, dataclasses, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))
import numpy as np

TOPICS = {"enhanced": ("eagleg7", "enhanced"), "pcl": ("eagleg7", "pcl"), "ars548": ("ars548", "points")}
RCS_NAMES, DOPPLER_NAMES = ("Power", "intensity"), ("Doppler", "doppler")

def read_snail_pcd(path):
    with open(path, "rb") as f:
        h = {}
        while True:
            line = f.readline().decode("ascii").strip()
            k, *v = line.split(); h[k] = v
            if k == "DATA": break
        assert h["DATA"][0] == "binary", path
        assert all(t == "F" for t in h["TYPE"]) and all(s == "4" for s in h["SIZE"]), path
        dt = np.dtype([(n, np.float32) for n in h["FIELDS"]]); n = int(h["POINTS"][0])
        return np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)

def write_pcd(path, arr):
    n = len(arr)
    head = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z Power Doppler\n"
            "SIZE 4 4 4 4 4\nTYPE F F F F F\nCOUNT 1 1 1 1 1\n"
            f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    with open(path + ".tmp", "wb") as f:
        f.write(head.encode("ascii")); f.write(np.ascontiguousarray(arr, dtype=np.float32).tobytes())
    os.replace(path + ".tmp", path)

def reve_static(points, params):
    """REVE inliers of one scan (static returns), with the estimator of DRG stage 1."""
    from drg.ego_velocity import soft_reve, doppler_residual
    kept, velocity, _ = soft_reve(points, params, params.seed)
    if len(kept) < 3:
        return kept, velocity
    return kept[np.abs(doppler_residual(kept, velocity)) < params.reve_inlier_threshold], velocity

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seq", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--topic", default="enhanced", choices=sorted(TOPICS))
    ap.add_argument("--reve-filter", action="store_true", help="keep only REVE inliers (static returns)")
    ap.add_argument("--min-rcs", type=float, default=None, help="RCS floor of the FoV mask used by the REVE filter")
    a = ap.parse_args()
    params = None
    if a.reve_filter:
        from drg import DRGParams
        params = DRGParams() if a.min_rcs is None else dataclasses.replace(DRGParams(), min_rcs=a.min_rcs)
    src = os.path.join(a.seq, *TOPICS[a.topic])
    names = [l.strip() for l in open(os.path.join(src, "times.txt")) if l.strip()]
    names.sort(key=lambda s: (int(s.split(".")[0]), int(s.split(".")[1])))
    # SNAIL's ars548 times.txt repeats some stamps (6 in these 5 sequences); write each scan once.
    # NOTE: dumps written before 19 Nov 2026 still contain the repeats; run_drg_chunk.py skips them.
    nd = len(names); names = [n for i, n in enumerate(names) if i == 0 or n != names[i - 1]]
    if len(names) < nd: print(f"{nd - len(names)} repeated stamps in times.txt dropped")
    os.makedirs(a.out, exist_ok=True)
    missing = 0; n_in, n_out = [], []
    for k, name in enumerate(names):
        p = os.path.join(src, name + ".pcd")
        if not os.path.exists(p): missing += 1; continue
        raw = read_snail_pcd(p)
        rcs = next(f for f in RCS_NAMES if f in raw.dtype.names); dop = next(f for f in DOPPLER_NAMES if f in raw.dtype.names)
        pts = np.column_stack([raw["x"], raw["y"], raw["z"], raw[rcs], raw[dop]]).astype(np.float32)
        note = ""
        if a.reve_filter:
            static, v = reve_static(pts, params)
            note = f"reve_filter kept {len(static)} of {len(pts)} (fov+static), v_reve {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n"
            n_in.append(len(pts)); pts = static
        d = os.path.join(a.out, "%06d" % k); os.makedirs(d, exist_ok=True)
        write_pcd(os.path.join(d, "cloud.pcd"), pts)
        sec, nsec = name.split(".")
        with open(os.path.join(d, "data"), "w") as f:
            f.write(f"stamp {int(sec)} {int(nsec)}\nsource {os.path.relpath(p, a.seq)}\n{note}")
        n_out.append(len(pts))
    msg = (f"{a.seq} [{a.topic}{' + REVE filter' if a.reve_filter else ''}]: {len(names)} stamps, {len(n_out)} frames written, "
           f"{missing} missing pcd, points/scan median {int(np.median(n_out))} (min {min(n_out)}, max {max(n_out)})")
    if n_in: msg += f"; REVE kept {np.sum(n_out) / np.sum(n_in):.1%} of all returns (median scan {int(np.median(n_in))} -> {int(np.median(n_out))})"
    print(msg)
    if missing: sys.exit(f"ERROR: {missing} frames listed in times.txt have no pcd")

if __name__ == "__main__":
    main()
