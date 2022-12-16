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

from d3_inventory_demo.objective import Objective
from d3_inventory_demo.robot_state import RobotState
from d3_inventory_demo.tf_broadcast_helper import tf_broadcast_helper
from d3_inventory_demo.path import set_target_position, pose_to_goal
from d3_apriltag import apriltag_odom

POINT_FILE_PATH = "/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/config/points.json"
OBJECTIVE_FILE_PATH = "/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/config/objectives.json"

WAIT_FOR_PC = False

# Points & objectives are populated during the startup routine
# Each objective must have a matching point[objective.name]
objectives = []
points = {}
state = None

# A whole bunch of objects we initialize in startup()
move_base_client = None
tf_broadcaster = None
tf_listener = None
state_pub = None
dmtx_count_pub = None
april_tag = None

def process_state(current_state, current_objective):
    # START
    if state_pub is not None:
        rospy.loginfo("Publishing state: " + state.name)
        rospy.loginfo("Objective: " + current_objective.name)
        state_pub.publish(state.name)

    if state is RobotState.STARTUP:
        startup()
    elif state is RobotState.DRIVE:
        drive(current_objective.name, current_objective.precise_goal)
    elif state is RobotState.SCAN:
        scan(current_objective.action_info["number_targets"])
    elif state is RobotState.TRACK:
        track(current_objective.action_info["duration"])
    elif state is RobotState.NOOP:
        nothing()
    elif state is RobotState.DONE:
        done()

# This state is likely mis-named.  It is actually the
# step where we re-calculate localization.
def localize():
    rospy.loginfo("Localizing...")
    # Run april-tag detection code to determine robot position
    current_pose = april_tag.get_pose()
    rospy.sleep(0.01)
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
    tf_broadcaster.broadcast_transform("map", "odom", map2odom_pos, map2odom_quat)
    rospy.sleep(0.01)

# Given a target point we will create and send a goal to move_base,
# then wait for the robot to reach that goal.
def drive(target_name, precise = True):
    global state, points
    if target_name not in points.keys():
        raise ValueError("Error - pathing to point that does not exist")

    target_point = points[target_name]
    rospy.loginfo(state)
    rospy.loginfo("target:" + str(target_point))

    if move_base_client is None:
        print("No move-base-client - skipping")
        return

    # Technically this code can stay - drive to the goal position
    local_target = target_point
    reached_destination = False
    pos_err_thresh = 0.075
    yaw_err_thresh = 0.02
    # debug - remove later
    while not reached_destination:
        goal = pose_to_goal(local_target)

        # Tell the robot to drive to the target
        move_base_client.send_goal(goal)
        wait = move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

        # run localize to determine where the robot is on the map.
        # to get BLT and BL updated accordingly
        localize()

        # Measure YAW offset between BLT and BL, must be less than 0.01 radians
        t = tf_listener.getLatestCommonTime("/base_link", "/" + target_name)
        bl_to_pt, bl_to_pt_quat = tf_listener.lookupTransform("/base_link", "/" + target_name, t)
        bl_to_pt_rotation = tf.transformations.euler_from_quaternion(bl_to_pt_quat)

        x_err = bl_to_pt[0]
        y_err = bl_to_pt[1]
        yaw_err = bl_to_pt_rotation[2]

        print("X,Y error: (" + str(x_err) + ", " + str(y_err) + ")")
        print("Yaw error: " + str(yaw_err))
        print("X,Y threshold: " + str(pos_err_thresh))
        print("Yaw threshold: " + str(yaw_err_thresh))

        fix_position = False
        fix_orientation = False
        # If X error is behind us, we don't want the robot to go in circles.  Cut our losses and keep the current position.
        if (abs(x_err) > pos_err_thresh) or (abs(y_err) > pos_err_thresh) and x_err > 0:
            fix_position = True
            pos_err_thresh += 0.05

        if abs(yaw_err) > yaw_err_thresh:
            fix_orientation = True
            yaw_err_thresh += 0.01

        t = tf_listener.getLatestCommonTime("/map", "/base_link")
        current_position, current_orientation = tf_listener.lookupTransform("/map", "/base_link", t)
        if fix_orientation:
            local_target.orientation = target_point.orientation
        else:
            local_target.orientation = geometry_msgs.msg.Quaternion(*current_orientation)

        if fix_position:
            local_target.position = target_point.position
        else:
            local_target.position = geometry_msgs.msg.Point(*current_position)

        # Path to new yaw goal and loop, otherwise proceed to scan
        if (fix_position or fix_orientation) and precise:
            print("Did not make it to the goal - trying again")
            print("Fix Orientation? " + str(fix_orientation))
            print("Fix Position? " + str(fix_position))
        else:
            print("Made it to the goal - continuing")
            reached_destination = True

    return move_base_client.get_result()


def scan(num_targets = 1):
    global state, WAIT_FOR_PC
    rospy.loginfo(state)
    dmtx_count_pub.publish(num_targets)
    rospy.loginfo("Num targets:" + str(num_targets))
    if WAIT_FOR_PC:
        rospy.loginfo("Waiting for response from visualizer")
        rospy.wait_for_message('/viz_resp', String)

def startup():
    global state, state_pub, dmtx_count_pub
    global april_tag, tf_broadcaster, tf_listener, move_base_client
    global points, objectives

    # The startup state is special - and is published inside of the state.
    state_pub = rospy.Publisher('/robot_state', String, latch=True, queue_size=5)
    dmtx_count_pub = rospy.Publisher('/dmtx_count', Int32, latch=True, queue_size=5)

    # Broadcast state information
    state_pub.publish(state.name)
    rospy.loginfo(state)

    # Set up apriltag for localization
    april_tag = apriltag_odom.apriltag_odom(3, "/back/imx390/camera_info", "/back/imx390/image_raw_rgb", "imx390_rear_temp_optical", "apriltag21")

    # Populate points && objectives for the demo:
    points = Objective.read_point_file(POINT_FILE_PATH)
    objectives = Objective.read_objective_file(OBJECTIVE_FILE_PATH)
    for obj in objectives:
        if obj.name not in points.keys():
            raise ValueError("WARNING: Objective " + obj.name + " has no corresponding point - removing...")
            objectives.remove(obj)


    tf_listener = tf.TransformListener()
    tf_broadcaster = tf_broadcast_helper()

    # We have delays between the broadcasts because if you don't you end up losing all but the last broadcast.
    for point in points:
        tf_broadcaster.broadcast_transform_pose("map", point, points[point])
        rospy.sleep(0.5)

    tf_broadcaster.broadcast_transform("map", "odom", [0.,0.,0.], [0.,0.,0.,1.])

    # The move base wait MUST happen after the transform is published
    rospy.loginfo("Waiting for move-base to come online")
    move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    move_base_client.wait_for_server()

    if WAIT_FOR_PC:
        rospy.loginfo("Waiting for PC to connect")
        while(state_pub.get_num_connections() == 0):
            pass

    localize()

def track(duration):
    global state
    rospy.loginfo(state)
    rospy.sleep(duration)

def nothing():
    global state
    rospy.loginfo(state)

def done():
    global state
    rospy.loginfo(state)
    exit(0)

if __name__ == '__main__':
    try:
        state = RobotState.STARTUP
        rospy.init_node('d3_inventory_controller')

        process_state(state, None)
        while not rospy.is_shutdown():
            for obj in objectives:
                state = RobotState.DRIVE
                process_state(state, obj)
                state = obj.action
                process_state(state, obj)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")

