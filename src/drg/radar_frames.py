"""NTU4DRadLM sensor frames.

The raw `/radar_enhanced_pcl` points are in the radar frame (x forward, y left, z up). The paper
registers them in the radar-centred body frame of 4DRadarSLAM: the ROTATION of its Radar->Livox
extrinsic is applied and the translation is dropped (preprocessing_nodelet_ntu.cpp), so the axes
are parallel to the Livox / ground-truth frame and the origin stays at the radar. The rotation
is 3.13 deg from identity.
"""
import numpy as np

# Extrinsic chain copied from 4DRadarSLAM (apps/preprocessing_nodelet_ntu.cpp).
_LIVOX_TO_RGB = np.array([
    [-0.006878330000, -0.999969000000, 0.003857230000, 0.029164500000],
    [-7.737180000000E-05, -0.003856790000, -0.999993000000, 0.045695200000],
    [0.999976000000, -0.006878580000, -5.084110000000E-05, -0.19018000000],
    [0, 0, 0, 1]])
_THERMAL_TO_RGB = np.array([
    [0.9999526089706319, 0.008963747151337641, -0.003798822163962599, 0.18106962419014],
    [-0.008945181135788245, 0.9999481006917174, 0.004876439015823288, -0.04546324090016857],
    [0.00384233617405678, -0.004842226763999368, 0.999980894463835, 0.08046453079998771],
    [0, 0, 0, 1]])
_RADAR_TO_THERMAL = np.array([
    [0.999665, 0.00925436, -0.0241851, -0.0248342],
    [-0.00826999, 0.999146, 0.0404891, 0.0958317],
    [0.0245392, -0.0402755, 0.998887, 0.0268037],
    [0, 0, 0, 1]])
_RADAR_AXES_TO_CAMERA_AXES = np.array(
    [[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)

RADAR_TO_LIVOX = (np.linalg.inv(_LIVOX_TO_RGB) @ _THERMAL_TO_RGB @ _RADAR_TO_THERMAL
                  @ _RADAR_AXES_TO_CAMERA_AXES)
R_RADAR_TO_BODY = RADAR_TO_LIVOX[:3, :3].copy()


def radar_to_body(points: np.ndarray) -> np.ndarray:
    """Rotate (N, 5) radar-frame points x, y, z, RCS, Doppler into the body axes (float32)."""
    out = points.astype(np.float32, copy=True)
    out[:, :3] = points[:, :3].astype(np.float64) @ R_RADAR_TO_BODY.T
    return out
