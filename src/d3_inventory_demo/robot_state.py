from enum import Enum,auto

class RobotState(Enum):
    # Startup state is only entered once
    STARTUP = auto()
    DRIVE = auto()
    SCAN = auto()
    TRACK = auto()
    NOOP = auto()
    DONE = auto()

    @classmethod
    def exists(cls, key):
        for enum in list(cls):
            if enum.name == key:
                return True
        return False
