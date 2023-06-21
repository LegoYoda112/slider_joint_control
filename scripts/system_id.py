#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from math import sin, cos
import numpy as np

import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('system_id')
        self.publisher = self.create_publisher(Float32MultiArray, '/slider/control/joint/position_goals', 10)
        
        self.joint_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )        

        self.t = 0

        self.amplitude_rad = 0.5

        self.start_frequency_hz = 0.1
        self.end_frequency_hz = 0.1

        self.sweep_time_s = 100.0

        self.joint_states = None

        # Wait until we have a joint state
        while(self.joint_states == None):
            rclpy.spin_once(self)
            print("waiting")
            
            pass

        for i in range(10):
            rclpy.spin_once(self)
            # time.sleep(0.001)
        
        print(self.joint_states.position)
        
        self.move_time = 2.0 # seconds
        self.move_steps = int(100.0 * self.move_time)

        self.initial_position = np.array(self.joint_states.position)

        starting_position = np.array([0.0, # Right_Roll
            0.0, # Right_Pitch
            0.0, # Right_Slide
            0.0, # Right_Foot_Roll
            0.0, # Right_Foot_Pitch

            0.0, # Left_Roll
            0.0, # Left_Pitch
            0.0, # Left_slide
            0.0, # Left_Foot_Pitch
            0.0]) # check for user input

        for t in range(0, self.move_steps):
            factor = 1 - t / self.move_steps
            # print(factor)
            new_states = (1.0 - factor) * starting_position + factor * self.initial_position
            print(new_states)

            self.pub_joint_states(new_states)
            time.sleep(0.01)


        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def joint_callback(self, msg):
        self.joint_states = msg
        # print(self.joint_states)
        pass
        # print(msg.data)

    def pub_joint_states(self, array):
        msg = Float32MultiArray()
        msg.data = list(array)

        self.publisher.publish(msg)
        self.get_logger().info("Publishing")

    def timer_callback(self):
        # print("loop op")
        # Increment relative timer
        self.t += self.timer_period

        # # Find current frequency
        # freq_hz = np.interp(self.t, 
        #     [0, self.sweep_time_s], 
        #     [self.start_frequency_hz, self.end_frequency_hz])

        # # freq_hz = 2
        # freq_rads = freq_hz * 2 * np.pi

        # # Get target angle
        # angle = self.amplitude_rad * sin(self.t * freq_rads)
        # length = 0.05 * sin(self.t * freq_rads)

        if(round(self.t) % 2):
            target = 0.04
        else:
            target = 0.0

        # print(round(self.t))
        

        msg = Float32MultiArray()
        msg.data = [0.0, # Right_Roll
                    0.0, # Right_Pitch
                    0.01 - 0.00 * sin(self.t * 1.0) + target, # Right_Slide
                    0.0 * sin(-self.t * 3.0), # Right_Foot_Roll
                    0.0 * cos(-self.t * 3.0), # Right_Foot_Pitch

                    0.0, # Left_Roll
                    0.0, # Left_Pitch
                    0.01 - 0.00 * sin(self.t * 1.0) + target, # Left_slide
                    0.0 * sin(self.t * 3.0), # Left_Foot_Roll
                    0.0 * cos(self.t * 3.0)] # Left_Foot_Pitch
        # print(self.initial_position)
        print(msg.data)


        # msg.data = list(self.initial_position)

        # If we have reached the end of the test, finish
        # if(self.t > self.sweep_time_s):
        #     self.get_logger().info("FINISHED")
        #     # Stop the timer
        #     self.timer.cancel()

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()