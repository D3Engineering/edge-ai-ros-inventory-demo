import json
from geometry_msgs.msg import Quaternion,Point,Pose
from d3_inventory_demo.robot_state import RobotState

class Objective():
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

    def __repr__(self):
        return f'Objective(name={self.name}, sequence={self.sequence}, action={self.action}, action_info={self.action_info}, point_name={self.point_name}, precise_goal={self.precise_goal})'

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

