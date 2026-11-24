#!/usr/bin/env python3
"""Publish DRG odometry on /odom so that the 4DRadarSLAM back end runs on DRG as its front end.

The stock stack computes scan-to-scan odometry in ScanMatchingOdometryNodelet and publishes
nav_msgs/Odometry on /odom, which RadarGraphSlamNodelet consumes time-synchronised with
/filtered_points. This node replaces that nodelet: for every /filtered_points message it looks up the
accumulated DRG pose for that exact stamp (built offline by scripts/slam/build_odom_table.py from the
stored per-pair transforms) and republishes it in the same message type on the same topic. Everything
downstream - keyframe selection, Scan Context loop detection, loop verification, g2o pose-graph
optimisation - is untouched, so the difference against the stock run is the front end and nothing else.

Stamps are matched exactly: the table is built from the dump written by this same pipeline, so every
/filtered_points stamp must be present. A stamp that is not found is counted and skipped, and the
count is reported at shutdown rather than silently ignored.

usage: python3 scripts/slam/odom_table_publisher.py _table:=<file> [_tol:=0.001]
"""
import rospy, numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class TableOdom(object):
    def __init__(self):
        table = rospy.get_param("~table")
        self.tol = float(rospy.get_param("~tol", 0.001))
        self.odom_frame = rospy.get_param("~odometry_frame", "odom")
        self.base_frame = rospy.get_param("~baselink_frame", "base_link")
        a = np.loadtxt(table, comments="#")
        self.stamps = a[:, 0]
        self.pose = a[:, 1:8]          # tx ty tz qx qy qz qw
        self.vel = a[:, 8:11]
        self.n_pub, self.n_miss = 0, 0
        rospy.loginfo("[drg_odom] %d poses from %s, span %.1f s, tol %.4f s",
                      len(self.stamps), table, self.stamps[-1] - self.stamps[0], self.tol)
        self.pub = rospy.Publisher("/odom", Odometry, queue_size=1000000)
        self.sub = rospy.Subscriber("/filtered_points", PointCloud2, self.cb, queue_size=1000000)
        rospy.on_shutdown(self.report)

    def cb(self, msg):
        t = msg.header.stamp.to_sec()
        i = int(np.argmin(np.abs(self.stamps - t)))
        if abs(self.stamps[i] - t) > self.tol:
            self.n_miss += 1
            if self.n_miss <= 5:
                rospy.logwarn("[drg_odom] no pose within %.4f s of %.6f (nearest %.6f)",
                              self.tol, t, self.stamps[i])
            return
        p, v = self.pose[i], self.vel[i]
        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = p[0], p[1], p[2]
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = p[3], p[4]
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = p[5], p[6]
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = v[0], v[1], v[2]
        self.pub.publish(o)
        self.n_pub += 1
        if self.n_pub % 1000 == 0:
            rospy.loginfo("[drg_odom] published %d, missed %d", self.n_pub, self.n_miss)

    def report(self):
        rospy.loginfo("[drg_odom] FINAL published %d odometry messages, %d clouds with no pose",
                      self.n_pub, self.n_miss)


if __name__ == "__main__":
    rospy.init_node("drg_odom_table_publisher")
    TableOdom()
    rospy.spin()
