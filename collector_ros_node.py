#!/usr/bin/env python3
import logging
import os
import time
from collections import defaultdict
from functools import partial

import rclpy
from rclpy.node import Node
from turtlesim.msg import Color, Pose

from config import QOS_PROFILE, SAMPLE_PERIOD_SECONDS
from memory import Memory

# TODO: set up configuration for a better logging system - CONFIG FILE FOR IP
# TODO: type everything later
# TODO: make TEST and NON TEST VERSIONS of everything!
# ASSUMPTION: all information provided from the server is valid
logger = logging.getLogger(__name__)

# TODO: set up normal config file for running code
# TODO: Make Topic and message handling more abstract


def turtle1_pose_handler(pose):
    print(f"Message: {pose}")
    print(Memory.get_agv())
    x, y, theta = pose.x, pose.y, pose.theta
    Memory.update_agv_state(x=x, y=y, theta=theta)


def turtle1_color_sensor_handler(message):
    pass


TOPICS = [
    {
        "topic": "/turtle1/pose",
        "msg_type": Pose,
        "msg_handler": turtle1_pose_handler,
    },
    {
        "topic": "/turtle1/color_sensor",
        "msg_type": Color,
        "msg_handler": turtle1_color_sensor_handler,
    },
]


# NOTE THAT THIS SCRIPT MUST ALWAYS RUN FIRST!!
class InformationRetrievalHandler:
    @classmethod
    def handle_message(cls, topic_name, message):
        handler_map = {topic_obj["topic"]: topic_obj["msg_handler"] for topic_obj in TOPICS}
        message_handler_function = handler_map.get(topic_name, None)
        if message_handler_function is None:
            raise Exception(
                f"Message for topic_name: {topic_name} does not have corresponding handler function"
            )
        # make a call to the handler function
        message_handler_function(message)


class InformationRetrievalSubscription:
    def __init__(self):
        self.last_called = None
        self.subscription = None


class InformationRetrievalNode(Node):
    def __init__(self):
        super().__init__(f"information_retrieval_node")
        ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
        print(f"Starting Information Retrieval for AGV with ROS_DOMAIN_ID:{ros_domain_id}")
        self.node_subscriptions = defaultdict(lambda: InformationRetrievalSubscription())
        Memory.wipe_database()
        Memory.create_agv_state()
        Memory.update_agv_state(id=ros_domain_id)

        # partial used to pass in the topic_name for the handler
        for topic in TOPICS:
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
            # sanity check
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
            InformationRetrievalHandler.handle_message(topic_name, message)
        except Exception as error:
            logger.error(
                f"InformationRetrievalHandler unable to handle message\nError Message: {error}"
            )


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = InformationRetrievalNode()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
