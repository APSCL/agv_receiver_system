import os
import threading
import time
from http import HTTPStatus

from geometry_msgs.msg import PoseStamped
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


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

def navigate_to_way_point(navigator, waypoint):
    # nav2 simple commander
    import pdb; pdb.set_trace()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = waypoint.x
    goal_pose.pose.position.y = waypoint.y
    goal_pose.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    # end

# define threading function up here! (to write that the current waypoint is visited!)
def send_navigation_proccess_to_background_test(navigator, next_waypoint):
    # get all current coordinates
    print(f"Navigating to! x:{next_waypoint.x} y:{next_waypoint.y}")
    navigate_to_way_point(navigator, next_waypoint)
    # nav_controller.navigate_to(current_x, current_y, current_theta, goal_x, goal_y)
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
            self.navigator = BasicNavigator()
            self.agv_set_initial_pose()
            self.navigation_function = send_navigation_proccess_to_background_test

        self.navigation_thread = None

    def _init_navigation(self):
        rclpy.init()

    def agv_set_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(initial_pose)

    def perform_waypoint_registration(self):
        Memory.update_agv_state(status=AGVState.READY)
        registration_successful = self.communicator.register_agv()
        # perform any other initialization tasks here
        return registration_successful

    def perform_action(self):
        agv = Memory.get_agv()
        agv_status = agv.status
        # perform "handshake" - STANDARD agv state update and recieve any information in return
        command = self.communicator.send_internal_state_update()
        # direct command processing from the server! (to be implemented)
        status_to_action = {
            AGVState.READY: self.perform_ready_action,
            AGVState.BUSY: self.perform_busy_action,
            AGVState.DONE: self.perform_done_action,
        }
        action_function = status_to_action[agv_status]
        action_result = action_function()
        # print(f"Result of {agv_status} function: {action_result}")
        # handle errors from action functions

    def perform_ready_action(self):
        status_code, task_json = self.communicator.get_task()
        
        if status_code != 200:
            # means there is no task to recieve, so we continue patiently waiting
            return AGVActionMessages.UNABLE_TO_RETRIEVE_TASK

        if self.parser.no_tasks_available(status_code):
            return AGVActionMessages.SUCCESS
        # import pdb; pdb.set_trace()
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
        Memory.update_task(id=next_task.id, status=TaskStatus.IN_PROGRESS)
        self.navigation_thread = threading.Thread(
            target=send_navigation_proccess_to_background_test,
            args=(self.navigator, next_task_waypoint),
        )
        self.navigation_thread.start()
        return AGVActionMessages.SUCCESS

    def perform_busy_action(self):
        # check if we finished visting our currernt objective waypoint
        next_task = Memory.get_next_task()
        next_task_waypoint = Memory.get_next_task_waypoint()
        if self.navigation_thread.is_alive():
            return AGVActionMessages.SUCCESS

        if next_task_waypoint == []:
            Memory.update_task(id=next_task.id, status=TaskStatus.COMPLETE)
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
    # if True:
    #     print("registration failure")
    #     return
    # we will be running data collection with ROS before hand, but we want to make sure the AGV's current position
    # time.sleep(INTER_ITERATION_PERIOD_SECONDS)
    while True:
        controller.perform_action()
        time.sleep(INTER_ITERATION_PERIOD_SECONDS)


if __name__ == "__main__":
    main()
