#!/bin/env python3
import rospy
import math
import tf
import actionlib
import json
import tf2_ros

import numpy as np
import geometry_msgs
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

# Points are populated during the startup routine
point_a = None
point_b = None
point_c = None
point_home = None
state = None

# A whole bunch of objects we initialize in startup()
move_base_client = None
map2odom_broadcaster = None
tf_listener = None
state_pub = None
dmtx_count_pub = None
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

# This state is likely mis-named.  It is actually the
# step where we re-calculate localization.
def path(target_point):
    global state, point_a, point_b, point_c, point_home
    # Run april-tag detection code to determine robot position
    current_pose = april_tag.get_pose()

    # Determine position where robot should be by querying map -> base-link-temp
    # (robot real position after april-tag detection)
    #t = tf_listener.getLatestCommonTime("/base_link_temp", "/map")
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
    # We think that it's supposed to be BLT * ODOM^-1 - but we believe that
    # tf_listener.lookupTransform() - though it says it's arguments are "target, source"
    # we think it's returning a result as if it were "source, target".
    blt_mat = np.linalg.inv(blt_mat)
    map2odom_mat = blt_mat @ odom_mat

    map2odom_quat = R.from_matrix(map2odom_mat[:3,:3]).as_quat()
    map2odom_pos = map2odom_mat[:3,3]

    # Publish result in map->odom transform - static transform 
    static_transformStamped = setup_static_transform("map", "odom", map2odom_pos, map2odom_quat)
    map2odom_broadcaster.sendTransform(static_transformStamped)

# Given a target point we will create and send a goal to move_base,
# then wait for the robot to reach that goal.
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
    global state, state_pub, dmtx_count_pub
    global april_tag, map2odom_broadcaster, tf_listener, move_base_client
    global point_a, point_b, point_c, point_home

    # The startup state is special - and is published inside of the state.
    state_pub = rospy.Publisher('/robot_state', String, latch=True, queue_size=5)
    dmtx_count_pub = rospy.Publisher('/dmtx_count', Int32, latch=True, queue_size=5)

    # Broadcast state information
    state_pub.publish(state.name)
    rospy.loginfo(state)

    # Set up apriltag for localization
    april_tag = apriltag_odom.apriltag_odom(5, "/back/imx390/camera_info", "/back/imx390/image_raw_rgb", "imx390_rear_temp_optical", "apriltag21")

    # Populate points for the demo:
    point_a = read_point_file("A")
    point_b = read_point_file("B")
    point_c = read_point_file("C")
    point_home = read_point_file("HOME")
    print(point_a)

    tf_listener = tf.TransformListener()
    map2odom_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = setup_static_transform("map", "odom", [0.,0.,0.], [0.,0.,0.,1.])
    map2odom_broadcaster.sendTransform(static_transformStamped)

    # The move base wait MUST happen after the transform is published
    rospy.loginfo("Waiting for move-base to come online")
    move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    move_base_client.wait_for_server()

    if WAIT_FOR_PC:
        rospy.loginfo("Waiting for PC to connect")
        while(state_pub.get_num_connections() == 0):
            pass

def done():
    global state
    rospy.loginfo(state)
    rospy.sleep(3)

def read_point_file(point_name):
    filename = "/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/config/points.json"

    # Read from file
    with open(filename) as f:
        data = f.read()

    js = json.loads(data)

    # Convert to pose
    result_pose = Pose()
    if point_name not in js.keys():
        return None

    if "orientation" not in js[point_name].keys():
        return None

    if "position" not in js[point_name].keys():
        return None

    result_pose.orientation.x = js[point_name]["orientation"]["x"]
    result_pose.orientation.y = js[point_name]["orientation"]["y"]
    result_pose.orientation.z = js[point_name]["orientation"]["z"]
    result_pose.orientation.w = js[point_name]["orientation"]["w"]

    result_pose.position.x = js[point_name]["position"]["x"]
    result_pose.position.y = js[point_name]["position"]["y"]
    result_pose.position.z = js[point_name]["position"]["z"]

    return result_pose

# Given a postiion & rotation (in quats) - returns a TransformStamped - ready for publishing
def setup_static_transform(source, target, position, quaternion):
    static_transformStamped = geometry_msgs.msg.TransformStamped()

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
    return static_transformStamped

if __name__ == '__main__':
    try:
        state = RobotState.STARTUP
        rospy.init_node('d3_inventory_controller')

        process_state(state)
        state = state.next()
        while not rospy.is_shutdown():
            rospy.loginfo("Publishing state: " + state.name)
            state_pub.publish(state.name)
            process_state(state)
            rospy.sleep(1)
            state = state.next()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

