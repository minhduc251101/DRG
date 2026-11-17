#!/usr/bin/env python3
"""DRG radar odometry: register every pair of consecutive 4D radar scans and chain the result.

Subscribes
    ~input  (sensor_msgs/PointCloud or PointCloud2, see ~cloud_type)   remapped to the radar topic
Publishes
    ~odometry          nav_msgs/Odometry   pose of the radar in ~odom_frame
    ~path              nav_msgs/Path
    ~cloud_registered  sensor_msgs/PointCloud2   current scan in ~odom_frame (intensity = RCS)
    TF ~odom_frame -> ~body_frame (if ~publish_tf)
Writes (if ~output_dir is set)
    stamp_est_traj.txt   chained trajectory, TUM format (evo_ape / evo_rpe)
    relative_poses.txt   per-pair estimates (scripts/evaluate_pairs.py)

Every scan is processed; none is dropped, because DRG registers consecutive scans. DRG runs
offline (about 0.4-5 s per pair on one core), so play the bag slowly (see README).
"""
import os
import time

# Single-threaded, as for the paper: with more threads (BLAS / OpenMP) the estimates change in
# the last digits (~1e-11 m). This must run before numpy and open3d are imported.
os.environ["OMP_NUM_THREADS"] = "1"

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from scipy.spatial.transform import Rotation
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud, PointCloud2, PointField

from drg import DRG, DRGParams, NormStats
from drg.evaluation import relative_pose_line, tum_line
from drg.io import stamp_to_sec
from drg.radar_frames import radar_to_body
from drg.ros_conversions import pointcloud2_to_array, pointcloud_to_array

CLOUD_TYPES = {"PointCloud": (PointCloud, pointcloud_to_array),
               "PointCloud2": (PointCloud2, pointcloud2_to_array)}


class DRGOdometryNode:
    def __init__(self):
        # Parameters
        self.drg = DRG(DRGParams.from_dict(rospy.get_param("~drg", {})),
                       NormStats.from_yaml(rospy.get_param("~norm_stats")))
        cloud_type = rospy.get_param("~cloud_type", "PointCloud")
        self.rotate_to_body = rospy.get_param("~rotate_to_body", True)
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.body_frame = rospy.get_param("~body_frame", "radar_body")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        output_dir = rospy.get_param("~output_dir", "")

        # State
        self.pose = np.eye(4)
        self.previous = None            # (stamp, PreparedScan) of the last scan
        self.n_scans = 0
        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.traj_file = self.rel_file = None
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
            self.traj_file = open(os.path.join(output_dir, "stamp_est_traj.txt"), "w")
            self.rel_file = open(os.path.join(output_dir, "relative_poses.txt"), "w")
            rospy.on_shutdown(self.close_files)

        # Publishers first, subscriber last
        self.odom_pub = rospy.Publisher("~odometry", Odometry, queue_size=100)
        self.path_pub = rospy.Publisher("~path", Path, queue_size=1, latch=True)
        self.cloud_pub = rospy.Publisher("~cloud_registered", PointCloud2, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        msg_type, self.to_array = CLOUD_TYPES[cloud_type]
        # queue_size=None: unbounded, so no scan is dropped while DRG is busy.
        self.sub = rospy.Subscriber("~input", msg_type, self.on_scan, queue_size=None, buff_size=2**24)

        p = self.drg.p
        rospy.loginfo(f"[DRG] listening on {self.sub.resolved_name} ({cloud_type}); "
                      f"tau_rcs={p.tau_rcs} dB, tau_e={p.tau_residual} m/s, trials={p.ransac_trials}, "
                      f"sigma_v={p.sigma_v} m/s, sigma_g={p.sigma_g} m")

    def on_scan(self, msg):
        stamp = stamp_to_sec(msg.header.stamp.secs, msg.header.stamp.nsecs)
        points = self.to_array(msg)
        if self.rotate_to_body:
            points = radar_to_body(points)

        tic = time.time()
        scan = self.drg.prepare(points)
        if self.previous is None:
            self.write_pose(stamp)
        else:
            prev_stamp, prev_scan = self.previous
            dt = stamp - prev_stamp
            if dt <= 0:
                rospy.logwarn(f"[DRG] non-increasing stamp ({dt:.4f} s), scan skipped")
                return
            result = self.drg.register(scan, prev_scan, dt)
            self.pose = self.pose @ result.T
            self.write_pose(stamp)
            if self.rel_file:
                self.rel_file.write(relative_pose_line(prev_stamp, stamp, result.T))
                self.rel_file.flush()
            status = "" if result.success else "  FAILED -> identity"
            rospy.loginfo(f"[DRG] scan {self.n_scans}: gated {result.n_gated}/{result.n_putative}, "
                          f"w_dop={result.fusion_weight:.2f}, {time.time() - tic:.2f} s{status}")
        self.previous = (stamp, scan)
        self.n_scans += 1
        self.publish(msg.header.stamp, points)

    def write_pose(self, stamp):
        if self.traj_file:
            self.traj_file.write(tum_line(stamp, self.pose))
            self.traj_file.flush()

    def publish(self, ros_stamp, points):
        x, y, z = self.pose[:3, 3]
        qx, qy, qz, qw = Rotation.from_matrix(self.pose[:3, :3]).as_quat()

        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = ros_stamp, self.odom_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.body_frame
        odom.pose.pose = pose.pose
        self.odom_pub.publish(odom)

        self.path.header.stamp = ros_stamp
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header = pose.header
            tf.child_frame_id = self.body_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
            tf.transform.rotation = o
            self.tf_broadcaster.sendTransform(tf)

        if self.cloud_pub.get_num_connections() > 0:
            xyz = points[:, :3] @ self.pose[:3, :3].T + self.pose[:3, 3]
            fields = [PointField(n, 4 * k, PointField.FLOAT32, 1)
                      for k, n in enumerate(("x", "y", "z", "intensity"))]
            cloud = point_cloud2.create_cloud(pose.header, fields, np.column_stack([xyz, points[:, 3]]))
            self.cloud_pub.publish(cloud)

    def close_files(self):
        for f in (self.traj_file, self.rel_file):
            if f:
                f.close()
        rospy.loginfo(f"[DRG] processed {self.n_scans} scans")


if __name__ == "__main__":
    rospy.init_node("drg_odometry")
    DRGOdometryNode()
    rospy.spin()
