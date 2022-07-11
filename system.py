import os
import threading
import time
from http import HTTPStatus

import rclpy

from communications import (
    MockWaypointJSONParser,
    MockWaypointServerCommunicator,
    WaypointJSONParser,
    WaypointServerCommunicator,
)
from config import INTER_ITERATION_PERIOD_SECONDS
from core import AGVActionMessages, AGVState, MemoryAccessMessages, TaskStatus
from memory import AGV, Memory, Task, Waypoint
from ros_turtlesim_nav import TurtleSimNavigationPublisher


# define threading function up here! (to write that the current waypoint is visited!)
def send_navigation_proccess_to_background_test(nav_controller, next_waypoint):
    # get all current coordinates
    print(f"Navigating to! x:{next_waypoint.x} y:{next_waypoint.y}")
    agv = Memory.get_agv()
    current_x, current_y, current_theta = agv.x, agv.y, agv.theta
    goal_x, goal_y, waypoint_id = next_waypoint.x, next_waypoint.y, next_waypoint.id
    nav_controller.navigate_to(current_x, current_y, current_theta, goal_x, goal_y)
    # update the waypoint as visited (this will cause a change in the behavior of the AGV)
    Memory.update_waypoint(id=next_waypoint.id, visited=True)


def send_navigation_process_to_background(coordinates):
    pass


# TODO: make this into a graph datastructure
GRID = [(1, 10), (5.5, 10), (10, 10), (1, 5.5), (5.5, 5.5), (10, 5.5), (1, 1), (5.5, 1), (10, 1)]

# NEXT STEP (MOCK [COMMUNICATION CLASS] - THE SERVER and FEED IN TASKS - then do the server)


class AGVController:
    def __init__(self, testing=False):
        self._init_navigation()
        if testing:
            self.communicator = MockWaypointServerCommunicator()
            self.parser = MockWaypointJSONParser()
            self.navigator = TurtleSimNavigationPublisher()
            self.navigation_function = send_navigation_proccess_to_background_test
        else:
            self.communicator = WaypointServerCommunicator()
            self.parser = WaypointJSONParser()
            self.navigator = TurtleSimNavigationPublisher()
            self.navigation_function = send_navigation_proccess_to_background_test

        self.navigation_thread = None

    def _init_navigation(self):
        rclpy.init()

    def perform_waypoint_registration(self):
        Memory.update_agv_state(status=AGVState.READY)
        registration_successful, registration_response_message = self.communicator.register_agv()
        if not registration_successful:
            print(registration_response_message)
        return registration_successful

    def perform_action(self):
        agv = Memory.get_agv()
        agv_status = agv.status
        # perform "handshake" - STANDARD agv state update and recieve any information in return
        status_code, response_json = self.communicator.send_internal_state_update()
        if status_code is not HTTPStatus.OK:
            # implement error handling later
            print(response_json)
        # direct command processing from the server! (to be implemented)
        status_to_action = {
            AGVState.READY: self.perform_ready_action,
            AGVState.BUSY: self.perform_busy_action,
            AGVState.DONE: self.perform_done_action,
        }
        action_function = status_to_action[agv_status]
        action_result = action_function()
        print(f"Result of {agv_status} function: {action_result}")
        # handle errors from action functions

    def perform_ready_action(self):
        status_code, task_json = self.communicator.get_task()
        print("READY: ", status_code)

        if self.parser.no_tasks_available(status_code):
            return AGVActionMessages.SUCCESS

        if HTTPStatus.OK != status_code:
            return AGVActionMessages.UNABLE_TO_RETRIEVE_TASK

        self.parser.process_new_task(task_json)
        # start navigation towards the first waypoint, and background the navigation
        next_task = Memory.get_next_task()
        next_task_waypoint = Memory.get_next_task_waypoint()
        # if for some reason, the Task has no waypoints associated with it, we just mark the task as "complete"
        if next_task_waypoint is None:
            print("Task has no waypoints associated with it, proceeding to mark it as complete")
            Memory.update_task(id=next_task.id, status=TaskStatus.COMPLETE)
            Memory.update_agv_state(status=AGVState.DONE)
            return AGVActionMessages.SUCCESS

        # else mark the task as in progress and begin navigating to the next waypoint
        Memory.update_task(id=next_task.id, status=TaskStatus.IN_PROGRESS)
        self.navigation_thread = threading.Thread(
            target=send_navigation_proccess_to_background_test,
            args=(self.navigator, next_task_waypoint),
        )
        self.navigation_thread.start()

        return AGVActionMessages.SUCCESS

    def perform_busy_action(self):
        # check if we finished visting our currernt objective waypoint
        agv = Memory.get_agv()
        current_task = agv.current_task
        next_task_waypoint = Memory.get_next_task_waypoint()
        if self.navigation_thread.is_alive():
            return AGVActionMessages.SUCCESS

        if next_task_waypoint is None:
            Memory.update_task(id=current_task.id, status=TaskStatus.COMPLETE)
            Memory.update_agv_state(status=AGVState.DONE)
            return AGVActionMessages.SUCCESS

        # else navigate to the next waypoint
        self.navigation_thread = threading.Thread(
            target=self.navigation_function, args=(self.navigator, next_task_waypoint), daemon=True
        )
        self.navigation_thread.start()
        return AGVActionMessages.SUCCESS

    # reason for dividing these states: want to reduce the total amount of network calls per step!
    def perform_done_action(self):
        Memory.update_agv_state(status=AGVState.READY)
        Memory.update_agv_current_task(current_task_id=None)
        return AGVActionMessages.SUCCESS


def main():
    controller = AGVController(testing=False)
    # set local ROS_DOMAIN_ID variable to the running AGV - THIS IS NOW HANDLED IN THE CALLING SCRIPT
    # register AGV to the server
    resistration_success = controller.perform_waypoint_registration()
    if not resistration_success:
        return
    # we will be running data collection with ROS before hand, but we want to make sure the AGV's current position
    time.sleep(1)
    while True:
        controller.perform_action()
        time.sleep(INTER_ITERATION_PERIOD_SECONDS)


if __name__ == "__main__":
    main()
