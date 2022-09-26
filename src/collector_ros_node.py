#!/usr/bin/env python3
import logging
import os
import time
from collections import defaultdict
from functools import partial

import rclpy
from rclpy.node import Node
from turtlesim.msg import Color, Pose

from tf2_ros import TransformException 
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
 

from config import QOS_PROFILE, SAMPLE_PERIOD_SECONDS
from memory import Memory

logger = logging.getLogger(__name__)

class InformationRetrievalNode(Node):
    def __init__(self):
        # import pdb; pdb.set_trace()
        super().__init__(f"information_retrieval_node")
        ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
        print(f"Starting Information Retrieval for AGV with ROS_DOMAIN_ID:{ros_domain_id}")
        Memory.wipe_database()
        Memory.create_agv_state()
        Memory.update_agv_state(id=1)

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
        # TODO: send cord to server
        print("Current x: %.2f. Current Y: %.2f" % (self.current_x, self.current_y))   


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = InformationRetrievalNode()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
