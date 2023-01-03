from enum import Enum,auto

################
# robot_state.py is an enumeration class - used to organize
# the different potential robot states.
#
# The states should be self-explanatory:
# "STARTUP" -> when the robot is doing bootup-sequence stuff
# "DRIVE" -> when the robot is driving to an objective, or localizing
# "SCAN" -> when the robot is scanning for datamatrices
# "TRACK" -> when the robot is tracking using it's forward facing camera + radar
# "NOOP" -> does nothing (may or may not be broadcast) - will continue to the next objective
# "DONE" -> waits for input to continue
################

class RobotState(Enum):
    # Startup state is only entered once
    STARTUP = auto()
    DRIVE = auto()
    SCAN = auto()
    TRACK = auto()
    NOOP = auto()
    DONE = auto()

    # Check to see if the given enumeration exists in the class
    @classmethod
    def exists(cls, key):
        for enum in list(cls):
            if enum.name == key:
                return True
        return False
