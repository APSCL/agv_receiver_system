import time

from communications import WaypointServerCommunicator
from config import INTER_ITERATION_PERIOD_SECONDS
from core import AGVState, TaskStatus
from memory import AGV, InternalState, Task, Waypoint
from serailizers import Serializer


class AGVController:
    def __init__(self):
        self.memory = InternalState()

    def perform_waypoint_registration(self):
        # WaypointServerCommunicator.register_agv()
        print("In Registration")

    def perform_action(self):
        agv_status = self.memory.agv.status
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
        


def main():
    controller = AGVController()
    # register AGV to the server
    controller.perform_waypoint_registration()
    while True:
        controller.perform_action()
        time.sleep(INTER_ITERATION_PERIOD_SECONDS)
    controller.memory.close()

if __name__ == "__main__":
    main()
