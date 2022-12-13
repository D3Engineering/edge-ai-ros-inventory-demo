#!/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped, Point, Quaternion, TransformStamped

class tf_broadcast_helper:
    def __init__(self):
        self.tf_map = {}

    def broadcast_transform(self, source, target, position, quaternion):
        tf_key = source + "_" + target
        if tf_key not in self.tf_map.keys() or self.tf_map[tf_key] is None:
            self.tf_map[tf_key] = tf2_ros.StaticTransformBroadcaster()

        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = source
        static_transformStamped.child_frame_id = target

        static_transformStamped.transform.translation.x = position[0]
        static_transformStamped.transform.translation.y = position[1]
        static_transformStamped.transform.translation.z = position[2]

        static_transformStamped.transform.rotation.x = quaternion[0]
        static_transformStamped.transform.rotation.y = quaternion[1]
        static_transformStamped.transform.rotation.z = quaternion[2]
        static_transformStamped.transform.rotation.w = quaternion[3]
        print("Broadcasting " + source + " to " + target)
        self.tf_map[tf_key].sendTransform(static_transformStamped)

    def broadcast_transform_pose(self, source, target, pose):
        position = [pose.position.x, pose.position.y, pose.position.z]
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        self.broadcast_transform(source, target, position, quaternion)

