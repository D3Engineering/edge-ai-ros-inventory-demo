from enum import Enum,auto

class RobotState(Enum):
    # Startup state is only entered once
    STARTUP = auto()

    #PATHING_A = auto()
    DRIVING_A = auto()
    SCANNING_A = auto()

    #PATHING_B = auto()
    DRIVING_B = auto()
    SCANNING_B = auto()

    #PATHING_C = auto()
    DRIVING_C = auto()
    SCANNING_C = auto()

    #PATHING_HOME = auto()
    DRIVING_HOME = auto()
    DONE = auto()

    def next(self):
        next_state = None
        if self is not RobotState.DONE:
            next_state = RobotState(self.value+1)
        else:
            next_state = RobotState.DRIVING_A
        return next_state

    def prev(self):
        next_state = None
        if self is not RobotState.DRIVING_A:
            next_state = RobotState(self.value-1)
        else:
            next_state = RobotState.DONE
        return next_state
