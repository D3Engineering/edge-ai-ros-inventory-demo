#!/bin/env python3
################
# tf_broadcast_helper is a helper class that creates a static transform broadcaster.
#
# The purpose of the static transform broadcaster is to consolidate all static transform broadcasters in
# the demo script into one object.  We broadcast one static transform for each position calibrated in the demo
#
# The tf_broadcast_helper maintains a dictionary of transforms.  The dictionary key is comprised of
# the key "<source>+_+<target>" - thus you can only have one static broadcaster per source->target transform.
################

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped, Point, Quaternion, TransformStamped


class tf_broadcast_helper:
    def __init__(self):
        self.tf_map = {}

    # Create and store a new transform boradcaster, or update an old one
    # The user must specify the source -> target transform (E.G. Map -> point_a) (as strings)
    # additionally they must specify the transform translation & rotation, as vectors
    # Postiion = [x, y, z], rotation = [x ,y, z, w] (quaternions)
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

    # Same as the previous function - but takes a geometry_msgs `pose` instead of vectors.
    def broadcast_transform_pose(self, source, target, pose):
        position = [pose.position.x, pose.position.y, pose.position.z]
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        self.broadcast_transform(source, target, position, quaternion)

