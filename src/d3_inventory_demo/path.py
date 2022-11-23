#!/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf
import math

def quaternion_to_euler(pose_q):
    quaternion = (
        position.pose.pose.orientation.x,
        position.pose.pose.orientation.y,
        position.pose.pose.orientation.z,
        position.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # (roll, pitch, yaw)
    return euler

def euler_to_quaternion(pose_e):
    roll  = pose_e.x
    pitch = pose_e.y
    yaw   = pose_e.z
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    # (x, y, z, w)
    return quaternion

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    client.wait_for_server()

    goal = MoveBaseGoal()
    # get current position
    position = rospy.wait_for_message('/odom', Odometry)
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    pose_q = position.pose.pose.orientation
    pose_e = quaternion_to_euler(pose_q)
    print(pose_q)
    print(pose_e)
    # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
    # Find euler angles for easy computation

    goal.target_pose.pose.position.x = position.pose.pose.position.x + 1 * math.cos(yaw)
    goal.target_pose.pose.position.y = position.pose.pose.position.y + 1 * math.sin(yaw)
    goal.target_pose.pose.position.z = position.pose.pose.position.z

    goal.target_pose.pose.orientation.x = position.pose.pose.orientation.x
    goal.target_pose.pose.orientation.y = position.pose.pose.orientation.y
    goal.target_pose.pose.orientation.z = position.pose.pose.orientation.z
    goal.target_pose.pose.orientation.w = position.pose.pose.orientation.w

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

