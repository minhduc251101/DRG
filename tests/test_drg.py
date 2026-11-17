"""Unit tests on synthetic scans and on the tutorial window (no dataset needed):  python -m pytest tests/"""
import os
import sys

# Single-threaded, as for the paper: with more threads (BLAS / OpenMP) the estimates change in
# the last digits (~1e-11 m). This must run before numpy and open3d are imported.
os.environ["OMP_NUM_THREADS"] = "1"

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import json  # noqa: E402
from types import SimpleNamespace  # noqa: E402

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

from drg import DRG, DRGParams, NormStats  # noqa: E402
from drg.ego_velocity import doppler_residual, ransac_iterations, soft_reve  # noqa: E402
from drg.evaluation import GroundTruth, pose_error  # noqa: E402
from drg.fusion import doppler_translation, fusion_weight  # noqa: E402
from drg.io import DumpSequence  # noqa: E402
from drg.radar_frames import R_RADAR_TO_BODY  # noqa: E402
from drg.ros_conversions import pointcloud2_to_array, pointcloud_to_array  # noqa: E402
from drg.sequence import register_pairs  # noqa: E402
from drg.solver import weighted_svd  # noqa: E402

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

P = DRGParams()
STATS = NormStats(rcs=(12.0, 3.0), doppler_weight=(0.95, 0.06), density=(10.0, 12.0), height=(-0.5, 1.5))


def static_scene(n=3000, seed=0):
    """Random static structure in front of the radar: positions and RCS."""
    rng = np.random.default_rng(seed)
    xyz = np.column_stack([rng.uniform(3, 40, n), rng.uniform(-15, 15, n), rng.uniform(-1.5, 3, n)])
    return xyz, rng.uniform(5, 25, n)


def observe(world_xyz, rcs, T_world_sensor, velocity, noise=0.01, seed=1):
    """Scan of a static scene from a sensor at T_world_sensor moving with `velocity` (sensor frame)."""
    rng = np.random.default_rng(seed)
    R, t = T_world_sensor[:3, :3], T_world_sensor[:3, 3]
    xyz = (world_xyz - t) @ R                      # world -> sensor
    n = xyz / np.linalg.norm(xyz, axis=1, keepdims=True)
    doppler = -(n @ velocity) + rng.normal(0, noise, len(xyz))
    return np.column_stack([xyz, rcs, doppler]).astype(np.float32)


def test_ransac_iteration_count_matches_reve_default():
    assert ransac_iterations(0.4, 0.9999, 3) == 38


def test_soft_reve_recovers_velocity_and_flags_moving_returns():
    xyz, rcs = static_scene()
    v = np.array([5.0, 0.3, -0.1])
    scan = observe(xyz, rcs, np.eye(4), v)
    scan[:200, 4] += 2.0                            # 200 returns on a moving object
    kept, v_hat, w = soft_reve(scan, P, seed=42)
    assert np.allclose(v_hat, v, atol=0.01)
    static = slice(200, None)
    assert np.median(w[static]) > 0.9 and np.all(w[:200] < 1e-6)
    assert np.abs(doppler_residual(kept, v_hat)[static]).max() < 0.06


def test_soft_reve_at_standstill_gives_unit_weights():
    xyz, rcs = static_scene()
    _, v_hat, w = soft_reve(observe(xyz, rcs, np.eye(4), np.zeros(3)), P, seed=42)
    assert np.linalg.norm(v_hat) < P.zero_velocity and np.all(w == 1.0)


def test_fusion_weight_values_quoted_in_the_paper():
    assert fusion_weight(1 / 12, P) == pytest.approx(0.93, abs=0.005)   # 12 Hz
    assert fusion_weight(1.0, P) == pytest.approx(0.08, abs=0.005)      # 1 s keyframes
    assert fusion_weight(P.sigma_g / P.sigma_v, P) == pytest.approx(0.5)  # equal weight at 0.3 s


def test_doppler_translation_is_trapezoidal_integration():
    R = Rotation.from_euler("z", 10, degrees=True).as_matrix()
    v_s, v_t = np.array([2.0, 0, 0]), np.array([1.0, 0.5, 0])
    assert np.allclose(doppler_translation(v_s, v_t, R, 0.1), 0.05 * (v_t + R @ v_s))


def test_weighted_svd_recovers_a_rigid_transform():
    rng = np.random.default_rng(3)
    A = rng.normal(size=(3, 50))
    R = Rotation.from_euler("xyz", [3, -2, 25], degrees=True).as_matrix()
    t = np.array([[0.4], [-0.2], [0.05]])
    T = weighted_svd(A, R @ A + t, rng.uniform(0.1, 1, 50))
    assert np.allclose(T[:3, :3], R) and np.allclose(T[:3, 3:4], t)


def test_radar_to_body_rotation_is_a_small_proper_rotation():
    assert np.allclose(R_RADAR_TO_BODY @ R_RADAR_TO_BODY.T, np.eye(3), atol=1e-4)
    angle = np.degrees(np.linalg.norm(Rotation.from_matrix(R_RADAR_TO_BODY).as_rotvec()))
    assert angle == pytest.approx(3.13, abs=0.01)


def test_end_to_end_registration_of_two_synthetic_scans():
    """Two scans 1/12 s apart at 6 m/s: DRG must recover the motion to a few centimetres."""
    xyz, rcs = static_scene(seed=4)
    dt, v = 1 / 12, np.array([6.0, 0.0, 0.0])
    yaw = Rotation.from_euler("z", 1.0, degrees=True).as_matrix()
    T_world_source = np.eye(4)
    T_world_source[:3, :3], T_world_source[:3, 3] = yaw, v * dt     # target sensor at the origin
    target = observe(xyz, rcs, np.eye(4), v, seed=5)
    source = observe(xyz, rcs, T_world_source, yaw.T @ v, seed=6)

    drg = DRG(P, STATS)
    result = drg.register(drg.prepare(source), drg.prepare(target), dt)
    te, re = pose_error(result.T, T_world_source)
    assert result.success and te < 0.03 and re < 0.2
    assert result.fusion_weight == pytest.approx(0.93, abs=0.005)


def test_pointcloud_channels_are_read_by_name():
    """sensor_msgs/PointCloud as in /radar_enhanced_pcl: channels Doppler, Range, Power, Alpha, Beta."""
    xyz = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    msg = SimpleNamespace(
        points=[SimpleNamespace(x=x, y=y, z=z) for x, y, z in xyz],
        channels=[SimpleNamespace(name=n, values=v) for n, v in
                  [("Doppler", [-1.5, 0.5]), ("Range", [3.7, 8.8]), ("Power", [12.0, 7.5]),
                   ("Alpha", [0.1, 0.2]), ("Beta", [0.3, 0.4])]])
    expected = np.column_stack([xyz, [12.0, 7.5], [-1.5, 0.5]]).astype(np.float32)
    assert np.array_equal(pointcloud_to_array(msg), expected)


def test_pointcloud2_with_the_padded_pcl_layout_of_filtered_points():
    """4DRadarSLAM's /filtered_points is pcl::PointXYZINormal: 48-byte points with padding."""
    rng = np.random.default_rng(0)
    points = rng.normal(size=(7, 5)).astype(np.float32)             # x y z RCS Doppler
    offsets = {"x": 0, "y": 4, "z": 8, "intensity": 32, "normal_x": 16, "normal_y": 20,
               "normal_z": 24, "curvature": 36}
    buf = np.zeros((7, 12), np.float32)
    for column, name in enumerate(["x", "y", "z", "intensity", "curvature"]):
        buf[:, offsets[name] // 4] = points[:, column]
    msg = SimpleNamespace(
        fields=[SimpleNamespace(name=n, offset=o, datatype=7, count=1) for n, o in offsets.items()],
        point_step=48, width=7, height=1, is_bigendian=False, data=buf.tobytes())
    assert np.array_equal(pointcloud2_to_array(msg), points)


def test_tutorial_window_reproduces_the_paper():
    """The first pairs of example/data (windowed set loop3 set 4) against the paper's estimates."""
    seq = DumpSequence(os.path.join(REPO, "example", "data", "loop3_set4"))
    gt = GroundTruth(os.path.join(REPO, "example", "data", "loop3_gt_odom_excerpt.txt"))
    drg = DRG(P, NormStats.from_yaml(os.path.join(REPO, "config", "norm_stats", "loop3.yaml")))
    with open(os.path.join(REPO, "tests", "reference", "windowed_sets_seed42.json")) as f:
        paper = {p["target"]: p for p in json.load(f)["sets"]["loop3_set4_5692"]["pairs"]}

    frames = seq.indices()[:4]
    for row in register_pairs(drg, seq, list(zip(frames[:-1], frames[1:])), gt):
        expected = paper[row["target"]]
        assert np.abs(row["T"] - np.array(expected["T"])).max() < 1e-9
        assert row["te"] == pytest.approx(expected["te"], abs=1e-9)
