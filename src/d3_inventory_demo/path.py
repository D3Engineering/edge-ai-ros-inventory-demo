#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import tf
import math

def forward():
    position = rospy.wait_for_message('/odom', Odometry)
    print(position)
    return position

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    client.wait_for_server()

    goal = MoveBaseGoal()
    # get current position
    position = rospy.wait_for_message('/odom', Odometry)
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Find euler angles for easy computation
    quaternion = (
        position.pose.pose.orientation.x,
        position.pose.pose.orientation.y,
        position.pose.pose.orientation.z,
        position.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

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

