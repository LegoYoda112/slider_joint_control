#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from math import sin, cos

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/slider/control/joint/position_goals', 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        self.i += 0.06


        tilt_forward = 0.08 * cos(self.i)
        tilt_sideways = 0.08 * sin(self.i)
        slide_height = 0.1
        msg = Float32MultiArray()
        msg.data = [0.0 + 0.0 * tilt_sideways, # Right_Roll
                    -(0.1 + 0.0 * tilt_forward), # Right_Pitch
                    slide_height, # Right_Slide
                    0.0 + tilt_forward, # Right_Foot_Roll
                    0.0 + tilt_sideways, # Right_Foot_Pitch

                    0.0 + 0.0 * tilt_sideways, # Left_Roll
                    0.1 + 0.0 * tilt_forward, # Left_Pitch
                    slide_height, # Left_slide
                    0.0 + tilt_forward, # Left_Foot_Pitch
                    0.0 + tilt_sideways]
        self.publisher.publish(msg)
        self.get_logger().info("Publishing")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()