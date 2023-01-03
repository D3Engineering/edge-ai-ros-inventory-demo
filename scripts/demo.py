#!/bin/env python3

############################
# demo.py is the `main method` of the robot demo.
#
# This script allows the robot to path around a small course using odometry + an april-tag for localization,
# scan shelves for items, and track the crowd with radar-camera fusion.
#
############################
# Code Overview
#
# The robot operates at two levels: at a high level - it uses "Objectives" determines it's overall plan,
# at a lower level "states" determine what to do next:
#
# "Objectives" are the combination of a target position, plus an action to do (see objective.py for details)
# "States" are the individual actions, such as driving and scanning, etc. (see robot_state.py for all states)
#
# The core loop of the robot is as follows:
#
# For each objective:
#    Drive to the objective's target position (using move-base)
#    Localize the robot (using the over-head apriltag)
#    Perform objective action (typically via the visualizer)
#
############################
# Demo Constraints:
#
# The following things must be set up before the demo is run:
#
# * The scene must have a singular apriltag that is visible at all times
#       during the demo. We use a large tag placed high off the ground + an upward-facing camera
# * The objectives.json file should be configured to match your demo scenario
# * The robot must have had all points referenced in objectives.json calibrated
#
# The following is optional, but the demo will be incomplete without it:
# * a PC must be set up on the same network as the robot such that it can
#       run the visualization code.
#
############################
# Launch Parameters:
#
# point_file: The file which contains the results from the point-calibration script.
#             the robot should be calibrated such that there is a point for each unique objective
#             location.
#             DEFAULT: "/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/config/points.json"
#
# objective_file: The file which contains all of the objectives for the robot.
#             DEFAULT: "/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/config/objectives.json"
#
# wait_for_pc: Determines whether the robot will wait for a response from the PC or just keep driving
#              if TRUE then the robot will wait for the PC when starting up and before leaving a SCAN site.
#              DEFAULT:True
#
# num_poses: The number of pictures the robot will capture from it's pose-camera before
#            estimating it's current position with respect to the april tag
#             DEFAULT: 3
#
############################

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

# ROS parameters
POINT_FILE_PATH = None
OBJECTIVE_FILE_PATH = None
WAIT_FOR_PC = None
NUM_POSES = None

# Points & objectives are populated during the startup routine
# Each objective must have a matching point[objective.point_name]
objectives = []
points = {}

# Global state variable
state = None

# A whole bunch of objects we initialize in startup():
# For sending move-base goals
move_base_client = None

# for broadcasting static transforms
tf_broadcaster = None

# for receiving transform data
tf_listener = None

# for publishing states
state_pub = None

# for publishing the number of datamatrices expected to the visualizer
dmtx_count_pub = None

# for estimating pose using apriltags
april_tag = None

#################
# Given a current state and objective, will issue the next command to the robot.
# This function lets us map "states" to particular robot actions.
#################
def process_state(current_state, current_objective):
    # If we're in startup - state_pub is empty, so we can't publish yet
    # In the event of a NOOP, we don't publish that state because we're just
    # going to drive to the next location immediately anyway.
    if state_pub is not None or current_state is RobotState.NOOP:
        rospy.loginfo("Publishing state: " + state.name)
        rospy.loginfo("Objective: " + current_objective.name)
        state_pub.publish(current_objective.name + "|" + state.name)

    if state is RobotState.STARTUP:
        startup()
    elif state is RobotState.DRIVE:
        drive(current_objective.point_name, current_objective.precise_goal)
    elif state is RobotState.SCAN:
        scan(current_objective.action_info["number_targets"])
    elif state is RobotState.TRACK:
        track(current_objective.action_info["duration"])
    elif state is RobotState.NOOP:
        nothing()
    elif state is RobotState.DONE:
        done()

#################
# Localize the robot - updates map->odom transformation so that
# the robot position matches the position given by the apriltag.
# This function will stop the robot for long enough to capture
# the configured number of poses from the overhead apriltag.
#
# This function is not a state! instead, it's run after each move_base goal.
#################
def localize():
    rospy.loginfo("Localizing...")

    # Run april-tag detection code to determine robot position
    current_pose = april_tag.get_pose()
    # This sleep gives the system just enough time for the static transform
    # publisher to broadcast the latest transform.
    rospy.sleep(0.01)

    # Determine position where robot should be by querying map -> base-link-temp
    # (robot real position after april-tag detection)
    t = tf_listener.getLatestCommonTime("/map", "/base_link_temp")
    blt_position, blt_quaternion = tf_listener.lookupTransform("/base_link_temp", "/map", t)
    # Put the data into a Rotation + Position matrix
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
    # This is correct.
    # We believe that mathematically that it's supposed to be BLT * ODOM^-1 - but we think that
    # tf_listener.lookupTransform() is wrong. Though the documentation says it's arguments are
    # "target, source" we think it's returning a result as if it were "source, target".
    # so potentially we're really doing BLT^(-1*-1) * ODOM^-1
    blt_mat = np.linalg.inv(blt_mat)
    map2odom_mat = blt_mat @ odom_mat

    # convert the resulting matrix back into a position + quaternion vector.
    map2odom_quat = R.from_matrix(map2odom_mat[:3,:3]).as_quat()
    map2odom_pos = map2odom_mat[:3,3]

    # Publish result in map->odom transform - static transform 
    tf_broadcaster.broadcast_transform("map", "odom", map2odom_pos, map2odom_quat)
    rospy.sleep(0.01)

#################
# Drive state action
#
# Given a target point - drive() will create a move-base-goal and send it to move_base,
# then wait for the robot to reach that goal.  Upon reaching its' destination, it will
# rerun localize() to correct for any odometry error.
#
# If precise goal is true, then after localizing it will continue to drive
# to the original target point until it meets its' position / rotational requirements.
#################
def drive(target_name, precise = True):
    global state, points
    if target_name not in points.keys():
        raise ValueError("Error - pathing to point that does not exist")

    # Retrieve the point from the point-map
    target_point = points[target_name]
    rospy.loginfo(state)
    rospy.loginfo("target:" + str(target_point))

    if move_base_client is None:
        print("No move-base-client - skipping")
        return

    # Set some initial threshholds for error
    local_target = target_point
    reached_destination = False
    pos_err_thresh = 0.075
    yaw_err_thresh = 0.02

    # Main loop: Drive to goal -> re-localize -> determine error -> loop (if needed)
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

        # Measure YAW offset between BL and the target point, must be
        # less than the error threshhold (in radians)
        t = tf_listener.getLatestCommonTime("/base_link", "/" + target_name)
        bl_to_pt, bl_to_pt_quat = tf_listener.lookupTransform("/base_link", "/" + target_name, t)
        bl_to_pt_rotation = tf.transformations.euler_from_quaternion(bl_to_pt_quat)

        # Retrieve the X / Y position error, and the yaw error
        x_err = bl_to_pt[0]
        y_err = bl_to_pt[1]
        yaw_err = bl_to_pt_rotation[2]

        print("X,Y error: (" + str(x_err) + ", " + str(y_err) + ")")
        print("Yaw error: " + str(yaw_err))
        print("X,Y threshold: " + str(pos_err_thresh))
        print("Yaw threshold: " + str(yaw_err_thresh))

        # We have two different kinds of goal correction:
        # positionally correct and rotationally correction
        # generally, this is so that if your position was OK-ish, but your rotation was crap,
        # we can rotate in place and maintain the current position.
        fix_position = False
        fix_orientation = False

        # Check if it's within it's close to it's xy goal after localizing.  If it's not, correct it and increase the threshhold
        # If the robot drove past it's target point, only drive backwards if it's way past the point.
        # Note - given the nature of this condition, we are creating a square box of
        # size pos_err_thresh around the target point
        if ((abs(x_err) > pos_err_thresh) or (abs(y_err) > pos_err_thresh)) and (x_err > 0 or abs(x_err) > 2 * pos_err_thresh):
            fix_position = True
            pos_err_thresh += 0.05

        # Similarly, check if it's within the yaw error bound.
        # If not, flag it for correction increase the threshhold
        if abs(yaw_err) > yaw_err_thresh:
            fix_orientation = True
            yaw_err_thresh += 0.01

        # Retrieve robot current position - so we can potentially preserve our current orientation / position
        t = tf_listener.getLatestCommonTime("/map", "/base_link")
        current_position, current_orientation = tf_listener.lookupTransform("/map", "/base_link", t)

        # If orientation was flagged for correction, set the goal orientation to be the target point
        # if not, then set it to be the current orientation
        if fix_orientation:
            local_target.orientation = target_point.orientation
        else:
            local_target.orientation = geometry_msgs.msg.Quaternion(*current_orientation)

        # (Same as the orientation correction - but for position)
        if fix_position:
            local_target.position = target_point.position
        else:
            local_target.position = geometry_msgs.msg.Point(*current_position)

        # Final bit - if the goal was flagged as precise and we detected a discrepency
        # in either the position or orientation, we will re-run this driving loop.
        if (fix_position or fix_orientation) and precise:
            print("Did not make it to the goal - trying again")
            print("Fix Orientation? " + str(fix_orientation))
            print("Fix Position? " + str(fix_position))
        else:
            print("Made it to the goal - continuing")
            reached_destination = True

    # Likely outdated - but returns the result of the last move-base-goal exectued.
    return move_base_client.get_result()

#################
# Scan state action.  Will publish information so the visualizer PC can scan datamatrices.
# If WAIT_FOR_PC is true then the robot will wait for the PC to respond before continuing.
#
# num_targets indicates to the PC how many datamatrices it should expect at its' destination.
# in our testing we have found this parameter was not helpful so we leave it at 1.
#################
def scan(num_targets = 1):
    global state, WAIT_FOR_PC
    rospy.loginfo(state)
    dmtx_count_pub.publish(num_targets)
    rospy.loginfo("Num targets:" + str(num_targets))
    if WAIT_FOR_PC:
        rospy.loginfo("Waiting for response from visualizer")
        rospy.wait_for_message('/viz_resp', String)

#################
# Startup state action.  Sets up a lot of publishers and
# other objects for usage later in the demo.
#################
def startup():
    global state, state_pub, dmtx_count_pub
    global april_tag, tf_broadcaster, tf_listener, move_base_client

    # The startup state is special - it is published inside of the state.
    state_pub = rospy.Publisher('/robot_state', String, latch=True, queue_size=5)
    dmtx_count_pub = rospy.Publisher('/dmtx_count', Int32, latch=True, queue_size=5)

    # Broadcast state information - The visualizer expects "OBJECTIVE|STATE"
    # but since we're starting up we have no objective yet.
    state_pub.publish(state.name + "|" + state.name)
    rospy.loginfo(state)

    # Set up apriltag for localization
    april_tag = apriltag_odom.apriltag_odom(NUM_POSES, "/back/imx390/camera_info", "/back/imx390/image_raw_rgb", "imx390_rear_temp_optical", "apriltag21")

    # Populate objectives & points from their respective files.
    populate_objectives()

    tf_listener = tf.TransformListener()
    tf_broadcaster = tf_broadcast_helper()

    # We have delays between the broadcasts because if you don't you end up losing all but the last broadcast.
    for point in points:
        tf_broadcaster.broadcast_transform_pose("map", point, points[point])
        rospy.sleep(0.5)

    # Default map->odom publisher, so localize() won't break.
    tf_broadcaster.broadcast_transform("map", "odom", [0.,0.,0.], [0.,0.,0.,1.])

    # The move base wait MUST happen after the transform is published
    rospy.loginfo("Waiting for move-base to come online")
    move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    move_base_client.wait_for_server()

    if WAIT_FOR_PC:
        rospy.loginfo("Waiting for PC to connect")
        while(state_pub.get_num_connections() == 0):
            pass

    # Last step before doing anything - the robot must be localized
    localize()

#################
# Loads the POINT_FILE and OBJECTIVE_FILE into their respective global objects.  These
# files must have proper json formatting.  For details on how to populate them -see README.md
# on calibration and on objectives.
#
# points is a map from [point_name] -> Pose
# objectives is an ordered list of Objective objects.
#
# If for whatever reason you have an objective that maps to a point that
# doesn't exist, the program will quit
#################
def populate_objectives():
    global points, objectives
    # Populate points && objectives for the demo:
    points = Objective.read_point_file(POINT_FILE_PATH)
    objectives = Objective.read_objective_file(OBJECTIVE_FILE_PATH)
    for obj in objectives:
        if obj.point_name not in points.keys():
            raise ValueError("WARNING: Objective '" + obj.name + "' has no corresponding point named '" + obj.point_name + "' - removing...")
            objectives.remove(obj)

#################
# Track state action.  Track doesn't do anything
# from the robot perspective except sleep for the configured duration.
# the visualizer will display the radar+camera fusion data during this state
#################
def track(duration):
    global state
    rospy.loginfo(state)
    rospy.sleep(duration)

#################
# NOOP state action.
# Doesn't do anything at all.
#################
def nothing():
    global state
    rospy.loginfo(state)

#################
# DONE state action.  Waits for the user
# to press enter, at which point the demo will continue.
#################
def done():
    global state
    rospy.loginfo(state)
    input("PRESS ENTER TO CONTINUE")

#################
# Since most setup is handled in the STARTUP state
# this main method is just the parameter handling + core logic of the robot.
#################
if __name__ == '__main__':
    try:
        state = RobotState.STARTUP
        rospy.init_node('d3_inventory_controller')

        # Load parameters from launch file (or use defaults)
        POINT_FILE_PATH = rospy.get_param('~point_file', "/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/config/points.json")
        OBJECTIVE_FILE_PATH = rospy.get_param('~objective_file', "/opt/robotics_sdk/ros1/drivers/d3_inventory_demo/config/objectives.json")
        WAIT_FOR_PC = rospy.get_param('~wait_for_pc', True)
        NUM_POSES = rospy.get_param('~num_poses', 3)

        # Process startup state
        process_state(state, None)

        # Keep looping through objectives (Should really be checked after each objective...)
        while not rospy.is_shutdown():
            # Re-load objectives from files - this means you can update the robot on-the-fly
            # by changing the objectives.json file - the robot will respond after completing its' last objective
            populate_objectives()

            # Loop through each (ordered) objective:
            for obj in objectives:
                # First - drive to the objective's target location
                state = RobotState.DRIVE
                process_state(state, obj)

                # Next - perform the specific action at that target point
                state = obj.action
                process_state(state, obj)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")

