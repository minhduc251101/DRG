#!/usr/bin/env python3
"""Check a 4DRadarSLAM (Go-RIO) launch file against the configuration of results/slam_backend.

Passes if (1) no scan_matching_odometry_nodelet node is active (DRG publishes /odom instead), (2) gt_file_location is
not set and enable_gps is false (the back end must not read the ground truth), and (3) every parameter of the
preprocessing and graph-SLAM nodelets equals config/4dradarslam/backend_params.yaml (dataset_name and
enable_loop_closure, which is set per run, excepted).

    python3 scripts/slam/check_launch.py <your_drg_loop3.launch>
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

import yaml

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
SKIP = {"dataset_name", "enable_loop_closure"}


def same(a, b):
    try:
        return float(a) == float(b)
    except (TypeError, ValueError):
        return str(a).strip().lower() == str(b).strip().lower()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("launch")
    ap.add_argument("--params", default=os.path.join(REPO, "config", "4dradarslam", "backend_params.yaml"))
    args = ap.parse_args()

    text = re.sub(r"<!--.*?-->", "", open(args.launch).read(), flags=re.S)
    root = ET.fromstring(text)
    launch_args = {a.get("name"): a.get("default") for a in root.iter("arg") if a.get("default") is not None}
    resolve = lambda v: re.sub(r"\$\(arg ([^)]+)\)", lambda m: launch_args.get(m.group(1), "?"), v or "")
    nodes = {resolve(n.get("name")): {p.get("name"): resolve(p.get("value")) for p in n.findall("param")}
             for n in root.iter("node")}
    want = yaml.safe_load(open(args.params))
    problems = []
    if "scan_matching_odometry_nodelet" in nodes:
        problems.append("scan_matching_odometry_nodelet is active: remove its node block (DRG provides /odom)")
    slam = nodes.get("radar_graph_slam_nodelet", {})
    if slam.get("gt_file_location"):
        problems.append(f"gt_file_location is set ({slam['gt_file_location']}): remove it")
    for node, params in want.items():
        have = nodes.get(node)
        if have is None:
            problems.append(f"node {node} not found")
            continue
        for k, v in params.items():
            if k in SKIP:
                continue
            if k not in have:
                problems.append(f"{node}/{k}: missing (want {v})")
            elif not same(have[k], v):
                problems.append(f"{node}/{k}: {have[k]} (want {v})")
    if problems:
        print("\n".join(problems))
        sys.exit(f"{len(problems)} difference(s) from {os.path.relpath(args.params)}")
    print("OK: same configuration as results/slam_backend")


if __name__ == "__main__":
    main()
