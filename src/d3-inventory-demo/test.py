#!/bin/env python3
from robot_state import RobotState

num_targets_a = 1
num_targets_b = 2
num_targets_c = 3

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
    input("Press enter to continue")

def drive(target_point):
    global state
    print(state)
    print("target:" + str(target_point))
    input("Press enter to continue")

def scan(num_targets):
    global state
    print(state)
    print("Num targets:" + str(num_targets))
    input("Press enter to continue")

def startup():
    global state
    print(state)
    input("Press enter to continue")

def done():
    global state
    print(state)
    input("Press enter to continue")

if __name__ == '__main__':
    state = RobotState.STARTUP
    while state is not RobotState.DONE:
        process_state(state)
        state = state.next()

    process_state(state)
