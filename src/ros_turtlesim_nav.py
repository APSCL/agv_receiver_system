#!/usr/bin/env python3
import logging
import time
from distutils.log import debug

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

logger = logging.getLogger(__name__)

ANG_SPEED = (15) * 2 * np.pi / 360  # deg/sec
LINEAR_SPEED = 1.0
EPSILON = 0.001
DEBUG = 1

class TurtleSimNavigationPublisher(Node):
    """
    This class is used for communicating / controlling turtlesim bots as seen in the ROS beginners tutorial:
    https://ubuntu.com/tutorials/getting-started-with-ros-2#1-overview
    
    Team (2021 - 2022) Note: Navigation is often inaccurate within turtlesim, and that is perfectly fine. The
    point of using said environment is to verify that the Waypoint Server can successfully provide tasks and commands
    to AGVs, which are then executed / processed by the Receiver Software. Navigation accuracy within turtlesim is not
    crucial to this.
    """
    def __init__(self):
        super().__init__(f"turtle_sim_navigation_node")
        self.goal_queue = []
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def _init_twist_message(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        return vel_msg

    def _transform_rads_to_continuous_range(self, rad):
        """
        ROS theta (angular direction) is bounded between [-pi, pi]. In order to perform proper angular 
        arithmetic on ROS theta measurements it is easier for the mind to transform them to [0, 2pi] first
        """
        if rad < 0:
            return 2 * np.pi + rad
        return rad

    def navigate_to_waypoint(self, current_x, current_y, current_theta, goal_x, goal_y):
        # check if the provided coordinates are in the bounds of the turtlesim simulation environment map [0-10]
        # for x and y coordinates
        if 0 > goal_x or goal_x > 11 or goal_y < 0 or goal_y > 11:
            logger.warning(f"Invalid coordinate: ({goal_x},{goal_y})")

        # calculate the euclidean distance between the current postion and the destination
        diff_x, diff_y = goal_x - current_x, goal_y - current_y
        goal_distance = np.sqrt((diff_x) ** 2 + (diff_y) ** 2)

        # calculate the angular difference needed to travel in a straight line from the current position to the destination
        goal_theta = np.arctan2(diff_y, diff_x)
        goal_theta_t, current_theta_t = self._transform_rads_to_continuous_range(
            goal_theta
        ), self._transform_rads_to_continuous_range(current_theta)
        diff_theta = goal_theta_t - current_theta_t

        if DEBUG:
            print(f"Goal X: {goal_x}, Goal Y: {goal_y}")
            print(f"Diff X: {diff_x}, Diff Y: {diff_y} Diff Distance:{goal_distance}")
            print(f"Goal Theta: {goal_theta}, Current Theta: {current_theta}")
            print(f"Goal Theta T: {goal_theta_t}, Current Theta T: {current_theta_t}")
            print(
                f"Diff Theta T: {diff_theta}, Angular Speed: {float(np.sign(diff_theta) * ANG_SPEED)}"
            )

        # perform navigation
        self._rotate(diff_theta)
        self._move_forward(goal_distance, goal_theta)

    def _rotate(self, theta):
        vel_msg = self._init_twist_message()
        vel_msg.angular.z = float(np.sign(theta) * ANG_SPEED)

        t0 = time.time()
        current_angle = 0
        debug_counter = 0

        # rotate the turtlesim bot a 'theta' amount of degrees
        while 0 < abs(theta - current_angle) and abs(theta - current_angle) > (EPSILON):
            self.velocity_publisher.publish(vel_msg)
            t1 = time.time()
            current_angle = float(np.sign(theta) * ANG_SPEED) * (t1 - t0)
            if DEBUG and debug_counter == 3000:
                debug_counter = 0
                print(current_angle)
            debug_counter += 1

        # force the turtlesim bot to stop
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)

    def _move_forward(self, distance, theta):
        vel_msg = self._init_twist_message()
        vel_msg.linear.x = float(LINEAR_SPEED)

        t0 = time.time()
        current_distance = 0
        debug_counter = 0

        # translate the turtlesim bot a 'distance' amount of space
        while current_distance < distance:
            self.velocity_publisher.publish(vel_msg)
            t1 = time.time()
            current_distance = float(LINEAR_SPEED) * (t1 - t0)
            if DEBUG and debug_counter == 3000:
                debug_counter = 0
                print(current_distance)
            debug_counter += 1

        # force the turtlesim bot to stop
        vel_msg.linear.x = 0.0
        self.velocity_publisher.publish(vel_msg)
