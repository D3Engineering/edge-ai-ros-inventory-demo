from enum import Enum,auto

class RobotState(Enum):
    """
    Enumeration class - used to organize the different potential robot states.

    The states should be self-explanatory:
    "STARTUP" -> when the robot is doing bootup-sequence stuff
    "DRIVE" -> when the robot is driving to an objective, or localizing
    "SCAN" -> when the robot is scanning for datamatrices
    "TRACK" -> when the robot is tracking using it's forward facing camera + radar
    "NOOP" -> does nothing (may or may not be broadcast) - will continue to the next objective
    "DONE" -> waits for input to continue
    """
    STARTUP = auto()
    DRIVE = auto()
    SCAN = auto()
    TRACK = auto()
    NOOP = auto()
    DONE = auto()

    @classmethod
    def exists(cls, key):
        """
        Check to see if the given enumeration exists in the class

        :param cls: the current class
        :param key: the key of interest
        :return: whether the given enumeration is a valid RobotState
        """
        for enum in list(cls):
            if enum.name == key:
                return True
        return False
