#!/bin/env python3
import rospy
import math
import tf
import actionlib

import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Quaternion,Point,Pose

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from d3_inventory_demo.robot_state import RobotState
from d3_inventory_demo.path import set_target_position, pose_to_goal 
from d3_motorctl import apriltag_odom

WAIT_FOR_PC = False

num_targets_a = 6
num_targets_b = 6
num_targets_c = 6

point_a = [1,2,3]
point_b = [2,3,4]
point_c = [3,4,5]
point_home = [4,5,6]
state = None

move_base_client = None
map2odom_broadcaster = None
tf_listener = None
april_tag = None

def process_state(current_state):
    # START
    if state is RobotState.STARTUP:
        startup()

    # SHELF A:
    elif state is RobotState.PATHING_A:
        path(point_a)
    elif state is RobotState.DRIVING_A:
        drive(point_a)
    elif state is RobotState.SCANNING_A:
        scan(num_targets_a)

    # SHELF B:
    elif state is RobotState.PATHING_B:
        path(point_b)
    elif state is RobotState.DRIVING_B:
        drive(point_b)
    elif state is RobotState.SCANNING_B:
        scan(num_targets_b)

    # SHELF C:
    elif state is RobotState.PATHING_C:
        path(point_c)
    elif state is RobotState.DRIVING_C:
        drive(point_c)
    elif state is RobotState.SCANNING_C:
        scan(num_targets_c)

    # DRIVE HOME
    elif state is RobotState.PATHING_HOME:
        path(point_home)
    elif state is RobotState.DRIVING_HOME:
        drive(point_home)
    # DONE
    elif state is RobotState.DONE:
        done()

def path(target_point):
    global state, point_a, point_b, point_c, point_home
    # Run april-tag detection code to determine robot position
    current_pose = april_tag.get_pose()

    # Determine position where robot should be by querying map -> base-link-temp
    # (robot real position after april-tag detection)
    t = tf_listener.getLatestCommonTime("/map", "/base_link_temp")
    blt_position, blt_quaternion = tf_listener.lookupTransform("/base_link_temp", "/map", t)
    blt_mat = np.eye(4)
    blt_mat[:3, :3] = R.from_quat(blt_quaternion).as_matrix()
    blt_mat[:3, 3] = blt_position

    # Determine odometry -> base_link transform
    t = tf_listener.getLatestCommonTime("/odom", "/base_link")
    odom_position, odom_quaternion = tf_listener.lookupTransform("/base_link", "/odom", t)
    odom_mat = np.eye(4)
    odom_mat[:3, :3] = R.from_quat(odom_quaternion).as_matrix()
    odom_mat[:3, 3] = odom_position

    # BLT^-1 * ODOM
    # This is the correct one.  Don't touch it.
    blt_mat = np.linalg.inv(blt_mat)
    map2odom_mat = blt_mat @ odom_mat

    map2odom_quat = R.from_matrix(map2odom_mat[:3,:3]).as_quat()
    map2odom_pos = map2odom_mat[:3,3]

    # Publish result in map->odom transform
    current_time = rospy.Time.now()
    map2odom_broadcaster.sendTransform(
        map2odom_pos,
        map2odom_quat,
        current_time,
        "odom",
        "map"
    )

def drive(target_point):
    global state
    rospy.loginfo(state)
    rospy.loginfo("target:" + str(target_point))
    if move_base_client is None:
        print("No move-base-client - skipping")
        return

    # Technically this code can stay - drive to the goal position
    goal = pose_to_goal(target_point)

    print("Sending goal")
    move_base_client.send_goal(goal)
    print("Waiting for goal")
    wait = move_base_client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return move_base_client.get_result()


def scan(num_targets):
    global state, WAIT_FOR_PC
    rospy.loginfo(state)
    dmtx_count_pub.publish(num_targets)
    rospy.loginfo("Num targets:" + str(num_targets))
    if WAIT_FOR_PC:
        rospy.loginfo("Waiting for response from visualizer")
        rospy.wait_for_message('/viz_resp', String)

def startup():
    global state
    current_time = rospy.Time.now()
    map2odom_broadcaster.sendTransform(
        (0., 0., 0.),
        (0., 0., 0., 1.),
        current_time,
        "odom",
        "map"
    )
    rospy.loginfo(state)

def done():
    global state
    rospy.loginfo(state)


if __name__ == '__main__':
    try:
        rospy.init_node('d3_inventory_controller')

        map2odom_broadcaster = tf.TransformBroadcaster()
        tf_listener = tf.TransformListener()

        state_pub = rospy.Publisher('/robot_state', String, latch=True, queue_size=5)
        dmtx_count_pub = rospy.Publisher('/dmtx_count', Int32, latch=True, queue_size=5)
        state = RobotState.STARTUP

        april_tag = apriltag_odom.apriltag_odom(5, "/back/imx390/camera_info", "/back/imx390/image_raw_rgb", "imx390_rear_temp_optical", "apriltag21")

        rospy.loginfo("Waiting for move-base to come online")
        #move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #move_base_client.wait_for_server()

        if WAIT_FOR_PC:
            rospy.loginfo("Waiting for PC to connect")
            while(state_pub.get_num_connections() == 0):
                pass

        while True:
            rospy.loginfo("Publishing state: " + state.name)
            state_pub.publish(state.name)
            process_state(state)
            rospy.sleep(1)
            state = state.next()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

