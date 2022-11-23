#!/bin/env python3
import rospy
from d3_inventory_demo.robot_state import RobotState
from std_msgs.msg import String, Int32

num_targets_a = 6
num_targets_b = 6
num_targets_c = 6

point_a = [1,2,3]
point_b = [2,3,4]
point_c = [3,4,5]
point_home = [4,5,6]
state = None

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
    global state
    print(state)
    print("target:" + str(target_point))


def drive(target_point):
    global state
    print(state)
    print("target:" + str(target_point))
    print("Pretending to drive for 10 seconds")
    for i in range(4):
        print(str(4-i) + " seconds . . .")
        rospy.sleep(1)
    print("Done pretending to drive")

def scan(num_targets):
    global state
    print(state)
    dmtx_count_pub.publish(num_targets)
    print("Num targets:" + str(num_targets))
    print("Waiting for ACK")
    rospy.wait_for_message('/viz_resp', String)

def startup():
    global state
    print(state)

def done():
    global state
    print(state)


if __name__ == '__main__':
    try:
        rospy.init_node('d3_inventory_controller')
        state_pub = rospy.Publisher('/robot_state', String, latch=True, queue_size=5)
        dmtx_count_pub = rospy.Publisher('/dmtx_count', Int32, latch=True, queue_size=5)
        state = RobotState.STARTUP
        print("waiting for PC")
        while(state_pub.get_num_connections() == 0):
            pass

        while True:
            print("Publishing state: " + state.name)
            state_pub.publish(state.name)
            process_state(state)
            rospy.sleep(2)
            state = state.next()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

