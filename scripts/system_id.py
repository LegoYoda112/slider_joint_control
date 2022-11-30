#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

from math import sin, cos
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('system_id')
        self.publisher = self.create_publisher(Float32MultiArray, '/slider/control/joint/position_goals', 10)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.t = 0

        self.amplitude_rad = 0.5

        self.start_frequency_hz = 1.0/100.0
        self.end_frequency_hz = 0.1

        self.sweep_time_s = 20

    def timer_callback(self):
        # Increment relative timer
        self.t += self.timer_period

        # Find current frequency
        freq_hz = np.interp(self.t, 
            [0, self.sweep_time_s], 
            [self.start_frequency_hz, self.end_frequency_hz])
        freq_rads = freq_hz * 2 * np.pi

        # Get target angle
        angle = self.amplitude_rad * sin(self.t * freq_rads)

        msg = Float32MultiArray()
        msg.data = [0.0, # Right_Roll
                    0.0, # Right_Pitch
                    0.0, # Right_Slide
                    0.0, # Right_Foot_Roll
                    angle, # Right_Foot_Pitch

                    0.0, # Left_Roll
                    0.0, # Left_Pitch
                    0.0, # Left_slide
                    0.0, # Left_Foot_Pitch
                    0.0]

        # If we have reached the end of the test, finish
        if(self.t > self.sweep_time_s):
            self.get_logger().info("FINISHED")
            # Stop the timer
            self.timer.cancel()

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