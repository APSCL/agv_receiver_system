from enum import Enum


class BaseEnum(Enum):
    def __str__(self):
        return self.value


class AGVState(BaseEnum):
    READY = "READY"
    BUSY = "BUSY"
    DONE = "DONE"


class TaskStatus(Enum):
    INCOMPLETE = "INCOMPLETE"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETE = "COMPLETE"
