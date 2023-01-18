#!/usr/bin/env python3
import logging
import os
import time
from collections import defaultdict
from functools import partial

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from turtlesim.msg import Pose

import config
from config import QOS_PROFILE, SAMPLE_PERIOD_SECONDS
from memory import Memory

logger = logging.getLogger(__name__)

class InformationRetrievalNode(Node):
    def __init__(self):
        super().__init__(f"information_retrieval_node")
        agv_id = config.AGV_ID
        print(f"Starting Information Retrieval for AGV with ROS_DOMAIN_ID:{agv_id}")
        Memory.wipe_database()
        Memory.create_agv_state()
        Memory.update_agv_state(id=agv_id)

        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.on_timer)
        self.current_x = 0.0
        self.current_y = 0.0 
        self.current_yaw = 0.0


    def on_timer(self):
        """
        Callback function.
        This function gets called at the specific time interval.
        """
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'
    
        trans = None
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        # Publish the 2D pose
        self.current_x = trans.transform.translation.x
        self.current_y = trans.transform.translation.y
        Memory.update_agv_state(x=self.current_x, y=self.current_y)
        print("Current x: %.2f. Current Y: %.2f" % (self.current_x, self.current_y))


# ==== SIMULATION (TURTLESIM AND GAZEBO) NODE CLASS DEFINITIONS BELOW === #

def simulation_pose_handler(pose):
    print(f"Message: {pose}")
    x, y, theta = pose.x, pose.y, pose.theta
    Memory.update_agv_state(x=x, y=y, theta=theta)

SIMULATION_TOPICS = [
        {
            "topic": "/turtle1/pose",
            "msg_type": Pose,
            "msg_handler": simulation_pose_handler,
        }
]

class SimulationInformationRetrievalHandler:
    @classmethod
    def handle_message(cls, topic_name, message):
        handler_map = {topic_obj["topic"]: topic_obj["msg_handler"] for topic_obj in SIMULATION_TOPICS}
        message_handler_function = handler_map.get(topic_name, None)
        if message_handler_function is None:
            raise Exception(
                f"Message for topic_name: {topic_name} does not have corresponding handler function"
            )
        # make a call to the handler function
        message_handler_function(message)


class SimulationInformationRetrievalSubscription:
    def __init__(self):
        self.last_called = None
        self.subscription = None


class SimulationInformationRetrievalNode(Node):
    def __init__(self):
        super().__init__(f"information_retrieval_node")
        agv_id = os.environ.get("ROS_DOMAIN_ID")
        print(f"Starting Information Retrieval for AGV with ROS_DOMAIN_ID:{agv_id}")
        self.node_subscriptions = defaultdict(lambda: SimulationInformationRetrievalSubscription())
        Memory.wipe_database()
        Memory.create_agv_state()
        Memory.update_agv_state(id=agv_id)

        # partial functions used to pass in the topic_name for the handler
        for topic in SIMULATION_TOPICS:
            topic_name, topic_type = topic["topic"], topic["msg_type"]
            try:
                subscription = self.create_subscription(
                    topic_type,
                    topic_name,
                    partial(self.callback, topic_name),
                    QOS_PROFILE,
                )
                self.node_subscriptions[topic_name].subscription = subscription
            except Exception as error:
                logger.error(
                    f"InformationRetrievalNode creation for topic:[{topic}] failed\n Error Message: {error}",
                    exc_info=True,
                )
                print(error)
                raise Exception(f"Could not establish subscription connection to topic: [{topic}]")
            logger.info(f"InformationRetrievalNode creation for topic:[{topic}] successful!")

    def should_sample_ros_information(self, topic_name):
        node_subscription = self.node_subscriptions.get(topic_name, None)
        if node_subscription is None:
            return False
        time_now = time.time()
        if node_subscription.last_called is None:
            node_subscription.last_called = time_now
            return True
        time_passed = time_now - node_subscription.last_called
        if time_passed > SAMPLE_PERIOD_SECONDS:
            node_subscription.last_called = time_now
            return True
        return False

    def callback(self, topic_name, message):
        # check if this callback should run with time
        if not self.should_sample_ros_information(topic_name):
            return
        try:
            SimulationInformationRetrievalHandler.handle_message(topic_name, message)
        except Exception as error:
            logger.error(
                f"InformationRetrievalHandler unable to handle message\nError Message: {error}"
            )   

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = InformationRetrievalNode() if not os.environ.get("TESTING", False) else SimulationInformationRetrievalNode()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
