import sys
import threading
from enum import Enum


class BaseEnum(Enum):
    def __str__(self):
        return self.name

class CommandTypes(BaseEnum):
    CANCEL_TASK = "CANCEL_TASK"
    CANCEL_AGV = "CANCEL_AGV"
    STOP_AGV = "STOP_AGV"
    START_AGV = "START_AGV"


class AGVState(BaseEnum):
    READY = "READY"
    BUSY = "BUSY"
    DONE = "DONE"
    STOPPED = "STOPPED"


class TaskStatus(BaseEnum):
    INCOMPLETE = "INCOMPLETE"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETE = "COMPLETE"

class AGVDriveTrainType(BaseEnum):
    MECANUM = "MECANUM"
    ACKERMANN = "ACKERMANN"


class AGVActionMessages(BaseEnum):
    SUCCESS = 1
    ERROR = -1
    UNABLE_TO_RETRIEVE_TASK = -2
    

class TracedThread(threading.Thread):
  """
  TracedThread enables us to manually kill threads from within the context
  of a different python script.
  """
  def __init__(self, *args, **keywords):
    threading.Thread.__init__(self, *args, **keywords)
    self.killed = False
 
  def start(self):
    self.__run_backup = self.run
    self.run = self.__run     
    threading.Thread.start(self)
 
  def __run(self):
    sys.settrace(self.globaltrace)
    self.__run_backup()
    self.run = self.__run_backup
 
  def globaltrace(self, frame, event, arg):
    if event == 'call':
      return self.localtrace
    else:
      return None
 
  def localtrace(self, frame, event, arg):
    if self.killed:
      if event == 'line':
        raise SystemExit()
    return self.localtrace
 
  def kill(self):
    self.killed = True
