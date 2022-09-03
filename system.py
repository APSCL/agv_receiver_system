import time

from communications import WaypointServerCommunicator
from config import INTER_ITERATION_PERIOD_SECONDS
from core import AGVState, TaskStatus
from memory import AGV, Memory, Task, Waypoint
from serailizers import Serializer

GRID = [(1, 10), (5.5, 10), (10, 10), (1, 5.5), (5.5, 5.5), (10, 5.5)(1, 1), (5.5, 1), (10, 1)]

# NEXT STEP (MOCK [COMMUNICATION CLASS] - THE SERVER and FEED IN TASKS - then do the server)


class AGVController:
    def __init__(self):
        pass

    def perform_waypoint_registration(self):
        # WaypointServerCommunicator.register_agv()
        print("In Registration")

    def perform_action(self):
        # print out the agv state
        agv = Memory.get_agv()
        print(agv)
        agv_status = agv.status
        # perform "handshake" - STANDARD agv state update and recieve any information in return
        status_to_action = {
            AGVState.READY: self.perform_ready_action,
            AGVState.BUSY: self.perform_busy_action,
            AGVState.DONE: self.perform_done_action,
        }
        action_function = status_to_action[agv_status]
        res = action_function()

    def perform_ready_action(self):
        print("In Ready Action")
        # ping server for a task
        # parse and respond to output
        # 1. Recieve a task
        # - Create task with waypoints in Internal State
        # - Assign the Task to the AGV
        # - Change and update the internal state of the AGV to BUSY
        # - Start moving towards the first waypoint
        # 2. No Task Recieved
        # - Do nothing and keep waiting patiently
        pass

    def perform_busy_action(self):
        print("In Busy Action")
        # if task cancelled - delete all other remaining waypoints through visitation
        # check if we are "close enough" to our current objective waypoint
        # if so mark current object waypoint as visited
        # check if there is another waypoint to visit
        # if yes
        # Navigate to the next waypoint [END]
        # if no
        # mark internal status as done
        pass

    # reason for dividing these states: want to reduce the total amount of network calls per step!
    def perform_done_action(self):
        print("In Done Action")
        # Internal Clean up for memory
        # Indicate to the server that the task is complete
        # Put the AGV in the Ready State Again
        pass

    # TODO: Adapt these messages for the system rather than memory!

    # @classmethod
    # def get_current_task_current_waypoint(cls, session=None):
    #     if not cls.current_task_exists():
    #         return None
    #     waypoints = cls.get_current_task_waypoints(only_unvisited=True)
    #     if waypoints:
    #         return waypoints[0]
    #     return None

    # @classmethod
    # @thread_safe_db_access
    # def current_task_exists(cls, session=None):
    #     agv = session.query(AGV).first()
    #     current_task = agv.current_task
    #     if current_task is None:
    #         return False
    #     return True

    # @classmethod
    # @thread_safe_db_access
    # def get_current_task_waypoints(cls, only_unvisited=False, session=None):
    #     if not cls.current_task_exists():
    #         return []
    #     agv = session.query(AGV).first()
    #     current_task_id = agv.current_task
    #     filter_kwargs = {"task_id": current_task_id}
    #     if only_unvisited:
    #         filter_kwargs["visited"] = False
    #     waypoints = (
    #         session.query(Waypoint).filter_by(**filter_kwargs).order_by(Waypoint.order.asc()).all()
    #     )
    #     session.expunge_all()
    #     return waypoints


def main():
    controller = AGVController()
    # register AGV to the server
    print("IM IN THE SYSTEM!!")
    controller.perform_waypoint_registration()
    while True:
        controller.perform_action()
        time.sleep(INTER_ITERATION_PERIOD_SECONDS)
    controller.memory.close()


if __name__ == "__main__":
    main()
