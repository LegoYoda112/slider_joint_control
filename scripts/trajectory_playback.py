#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from math import sin, cos
import numpy as np

import time
from readchar import readkey, key
import signal

def handler(signum, frame):
    raise Exception()

signal.signal(signal.SIGALRM, handler)

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_playback')
        self.publisher = self.create_publisher(Float32MultiArray, '/slider/control/joint/position_goals', 10)
        
        self.joint_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )        

        self.t = 0
        self.index = 0
        self.trajectory_end = 500

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
        
        self.move_time = 1.0 # seconds
        self.move_steps = int(100.0 * self.move_time)

        self.initial_position = np.array(self.joint_states.position)

        # self.initial_position = 
        self.trajectory_start = self.get_pos_from_index()

        # Move from initial state to begining of trajectory
        for t in range(0, self.move_steps):
            factor = 1 - t / self.move_steps
            new_states = (1.0 - factor) * self.trajectory_start + factor * self.initial_position

            self.pub_joint_states(new_states)
            time.sleep(0.01)


        # print(self.joint_states.position[2])

        self.get_logger().info("Starting the test")

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    def get_pos_from_index(self):
        pos = np.zeros(10)
        pos[3] = np.sin(self.index / 10) * 0.2

        return pos

    def joint_callback(self, msg):
        self.joint_states = msg
        pass

    def pub_joint_states(self, array):

        msg = Float32MultiArray()
        msg.data = list(array)

        self.publisher.publish(msg)
        self.get_logger().info("Publishing")

    def timer_callback(self):

        char = " "
        try:
            signal.setitimer(signal.ITIMER_REAL, 0.1)
            char = readkey()
            time.sleep(0.1)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        except Exception as err:
            pass
            #print(err)
            #print("timeout")

        if(char == '[' and self.index > 0):
            self.index -= 1
        elif(char == ']' and self.index < self.trajectory_end):
            self.index += 1
        print(self.index)
        print(self.get_pos_from_index())

        self.pub_joint_states(self.get_pos_from_index())


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()