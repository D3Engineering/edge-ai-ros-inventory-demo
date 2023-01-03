import json

################
# objective.py is where objectives are stored.  In the final demo, an objective is
# trying to answer the following questions:
#
# * Where should the robot go?
# * What should the robot do when it gets there?
# * How should the robot perform that action?
# * How should these objectives be ordered?
#
# The demo uses objectives as it's foundation for how the robot will behave.
# It iterates through a sorted list of objectives to determine where to drive and what to do
# when it gets there.
################

from geometry_msgs.msg import Quaternion,Point,Pose
from d3_inventory_demo.robot_state import RobotState

class Objective():
    # Initialize an objective object.
    # name: the human-readable name of the objective
    # point_name: <optional> - the name of the point the objective should
    #             map to. if empty defaults to "name" (always lower case!)
    # sequence: used to sort objectives into an ordered list
    # action: which robot state the robot should enter upon reaching the target point
    # action_info: dictionary containing action-specific information
    # precise_goal: if true, the robot will re-localize upon reaching the point and
    #               check to see if it's reasonably close to the goal before proceeding.
    #               if not it will keep driving to the point till it gets close enough.
    #               if false, it will move on to the next state when it gets to the point, regardless
    #               of how close it is.
    def __init__(self, name, sequence, action, action_info, point_name = None, precise_goal = True):
        self.name = name
        if point_name is None:
            self.point_name = name.lower()
        else:
            self.point_name = point_name.lower()
        self.sequence = sequence
        if not RobotState.exists(action):
            raise ValueError("ERROR - Action " + action + " doesn't exist in Robot State")
        self.action = RobotState[action]
        self.action_info = action_info
        self.precise_goal = precise_goal

    # Helper to make objectives printable
    def __repr__(self):
        return f'Objective(name={self.name}, sequence={self.sequence}, action={self.action}, action_info={self.action_info}, point_name={self.point_name}, precise_goal={self.precise_goal})'

    # Given a path to a json file, will return a list of sorted objectives.
    @staticmethod
    def read_objective_file(filename):
        # Read from file
        with open(filename) as f:
            data = json.load(f)

        objectives = []
        for obj_name in data.keys():
            new_objective = Objective(name=obj_name, **data[obj_name])
            objectives.append(new_objective)

        # Sort objects by sequence
        objectives.sort(key=lambda x: x.sequence, reverse=False)
        return objectives

    # Given a path to a json file, will return a dictionary of points.
    # The point names should be all lower-case.
    @staticmethod
    def read_point_file(filename):

        # Read from file
        with open(filename) as f:
            data = json.load(f)

        points = {}

        for point_name in data.keys():
            if "orientation" not in data[point_name].keys():
                print("Point name '" + point_name + " missing orientation data - skipping")
                continue

            if "position" not in data[point_name].keys():
                print("Point name '" + point_name + " missing position data - skipping")
                continue

            tmp_pose = Pose()
            tmp_pose.orientation = Quaternion(**data[point_name]["orientation"])
            tmp_pose.position = Point(**data[point_name]["position"])

            points[point_name] = tmp_pose

        return points

