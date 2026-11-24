#!/usr/bin/env python3
"""SNAIL-Radar ground truth (full PGO trajectory, or the TLS reference ref_tls_T_xt32.csv)
 -- full ground truth (full_trajs/<date>/dataN/utm50r_T_xt32.txt, TUM, pose of the
PandarXT-32 lidar in UTM50) -> pose of the Oculii radar frame, TUM, same stamps:

    utm_T_oculii(t) = utm_T_xt32(t) * inv(base_T_xt32) * base_T_oculii

base_T_xt32 / base_T_oculii come from the sequence's own tf_static.launch
(static_transform_publisher args 'x y z qx qy qz qw parent child' = parent_T_child).

  python snail_gt_to_radar.py --gt .../utm50r_T_xt32.txt --tf .../tf_static.launch --out gt_oculii.txt
"""
import argparse, re
import numpy as np
from scipy.spatial.transform import Rotation as R

def tf_from_launch(path, child):
    for m in re.finditer(r'args="([^"]+)"', open(path).read()):
        a = m.group(1).split()
        if len(a) >= 9 and a[8].lstrip("/") == child:
            T = np.eye(4); T[:3, 3] = [float(v) for v in a[0:3]]
            T[:3, :3] = R.from_quat([float(v) for v in a[3:7]]).as_matrix()
            assert a[7].lstrip("/") == "base_link", a
            return T
    raise SystemExit(f"{child} not in {path}")

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gt", required=True); ap.add_argument("--tf", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--sensor", default="oculii")
    a = ap.parse_args()
    X = tf_from_launch(a.tf, "PandarXT-32"); S = tf_from_launch(a.tf, a.sensor)
    xt32_T_sensor = np.linalg.inv(X) @ S
    d = np.loadtxt(a.gt, ndmin=2, delimiter="," if a.gt.endswith(".csv") else None, comments="#")[:, :8]
    with open(a.out, "w") as f:
        f.write(f"# world_T_{a.sensor} from {a.gt}; xt32_T_{a.sensor} from {a.tf}\n")
        for r in d:
            T = np.eye(4); T[:3, 3] = r[1:4]; T[:3, :3] = R.from_quat(r[4:8]).as_matrix()
            T = T @ xt32_T_sensor
            q = R.from_matrix(T[:3, :3]).as_quat()
            f.write(f"{r[0]:.9f} {T[0,3]:.6f} {T[1,3]:.6f} {T[2,3]:.6f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
    rv = R.from_matrix(xt32_T_sensor[:3, :3]).as_euler("xyz", degrees=True)
    print(f"{a.out}: {len(d)} poses; xt32_T_{a.sensor}: t={np.round(xt32_T_sensor[:3,3],3)} rpy(deg)={np.round(rv,2)}")

if __name__ == "__main__":
    main()
