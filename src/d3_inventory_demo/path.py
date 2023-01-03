#!/bin/env python3

################
# path.py is a helper file used for testing goal-setting and pathing for the robot.
#
# In the final demo, it is only used for `pose_to_goal` which just takes a target pose
# and returns a MoveBaseGoal, one that can be sent directly to MoveBase.
#
# The rest of this file can be used for testing simple goals (E.G, move in a straight line
# at angle X for distance Y).
################

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf
import math

# Helper function - converts quaternion vector to an euler vector
def quaternion_to_euler(pose_q):
    quaternion = (
        pose_q.x,
        pose_q.y,
        pose_q.z,
        pose_q.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # (roll, pitch, yaw)
    return euler

# Helper function - converst an euler vector to a quaternion vector
def euler_to_quaternion(pose_e):
    quaternion = tf.transformations.quaternion_from_euler(*pose_e)
    # (x, y, z, w)
    return quaternion

# Helper function - given a starting position - will create a target pose
# that has an offset of <distance> meters at <angle> angle.  The vector's orientation
# will point directly away from the origin point.
def set_target_position(start_pose, distance, angle = 0):
    pose_q = start_pose.orientation
    pose_e = quaternion_to_euler(pose_q)
    pose_e = list(pose_e)
    pose_e[2] += angle
    pose_e = tuple(pose_e)
    target_pose = start_pose
    target_pose.position.x = start_pose.position.x + distance * math.cos(pose_e[2])
    target_pose.position.y = start_pose.position.y + distance * math.sin(pose_e[2])

    target_pose.orientation = Quaternion(*euler_to_quaternion(pose_e))
    return target_pose

# Converts a given target pose into a move-base-goal.
def pose_to_goal(target_pose):
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = target_pose

    return goal


# Main method used for testing move base goals
def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    # get current position
    position = rospy.wait_for_message('/odom', Odometry)
    pose = position.pose.pose
    target_pose = set_target_position(pose, 1, 0)
    goal = pose_to_goal(target_pose)

    print("Sending goal")
    client.send_goal(goal)
    print("Waiting for goal")
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

