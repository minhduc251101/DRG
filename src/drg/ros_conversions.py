"""ROS point-cloud messages -> (N, 5) float32 arrays x, y, z, RCS, Doppler.

Duck-typed on the message fields, so it works with rospy messages and with messages read
offline (e.g. by the `rosbags` package); no ROS import is needed.

Supported inputs:
    sensor_msgs/PointCloud   channels "Power" (RCS) and "Doppler"   - raw /radar_enhanced_pcl
    sensor_msgs/PointCloud2  fields intensity|rcs|Power, curvature|doppler|Doppler
                             - e.g. /filtered_points of 4DRadarSLAM
"""
import numpy as np

from .io import DOPPLER_FIELDS, RCS_FIELDS

_POINTFIELD_FLOAT32 = 7


def _pick(names, candidates, what):
    for name in candidates:
        if name in names:
            return name
    raise KeyError(f"no {what} channel; expected one of {candidates}, got {sorted(names)}")


def pointcloud_to_array(msg) -> np.ndarray:
    """sensor_msgs/PointCloud with RCS and Doppler channels."""
    channels = {c.name: c.values for c in msg.channels}
    xyz = np.array([(p.x, p.y, p.z) for p in msg.points], dtype=np.float32).reshape(-1, 3)
    rcs = np.asarray(channels[_pick(channels, RCS_FIELDS, "RCS")], dtype=np.float32)
    doppler = np.asarray(channels[_pick(channels, DOPPLER_FIELDS, "Doppler")], dtype=np.float32)
    return np.column_stack([xyz, rcs, doppler])


def pointcloud2_to_array(msg) -> np.ndarray:
    """sensor_msgs/PointCloud2 with float32 x, y, z, RCS and Doppler fields."""
    fields = {f.name: f for f in msg.fields}
    wanted = ["x", "y", "z", _pick(fields, RCS_FIELDS, "RCS"), _pick(fields, DOPPLER_FIELDS, "Doppler")]
    for name in wanted:
        if fields[name].datatype != _POINTFIELD_FLOAT32:
            raise TypeError(f"field {name} is not float32")
    dtype = np.dtype({"names": wanted, "formats": [np.float32] * 5,
                      "offsets": [fields[n].offset for n in wanted], "itemsize": msg.point_step})
    endian = ">" if msg.is_bigendian else "<"
    raw = np.frombuffer(bytes(msg.data), dtype=dtype.newbyteorder(endian),
                        count=msg.width * msg.height)
    return np.column_stack([raw[n].astype(np.float32) for n in wanted])
