from enum import Enum


class BaseEnum(Enum):
    def __str__(self):
        return self.name


class AGVState(BaseEnum):
    READY = "READY"
    BUSY = "BUSY"
    DONE = "DONE"


class TaskStatus(BaseEnum):
    INCOMPLETE = "INCOMPLETE"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETE = "COMPLETE"


class AGVActionMessages(BaseEnum):
    SUCCESS = 1
    ERROR = -1
    UNABLE_TO_RETRIEVE_TASK = -2


class MemoryAccessMessages(BaseEnum):
    ERROR = -1
    OBJECT_NOT_FOUND = -2
    NO_REMAINING_WAYPOINTS = -3


class WaypointServerEndpoints(BaseEnum):
    AGV_REGISTRATION = "/agvs/"
