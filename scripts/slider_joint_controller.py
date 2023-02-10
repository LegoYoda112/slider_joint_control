#!/usr/bin/env python3 

# Import ROS2 packages
import rclpy
from rclpy.node import Node

# Import ROS2 messages
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# Import Ankle kinematics
import ankle.ForwardKinematics
import ankle.InverseKinematics

class SliderJointController(Node):

    def __init__(self):
        super().__init__('slider_joint_controller')

        self.get_logger().info("Starting joint controller")
        
        # Make publishers
        self.position_publisher = self.create_publisher(Float32MultiArray, '/slider/control/motor/position_goals', 10)
        self.torque_publisher = self.create_publisher(Float32MultiArray, '/slider/control/motor/torque_goals', 10)

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Make subscribers
        self.position_subscriber = self.create_subscription(
            Float32MultiArray, 
            '/slider/control/joint/position_goals', 
            self.position_goal_callback,
            10)

        self.torque_subscriber = self.create_subscription(
            Float32MultiArray, 
            '/slider/control/joint/torque_goals', 
            self.torque_goal_callback,
            10)

        self.motor_state_subscriber = self.create_subscription(
            JointState, 
            'slider/control/motor/states', 
            self.motor_state_callback,
            10)

        # Transmission ratio between slide motor and slide joint
        # = pulley pitch diameter?
        # TODO: do exactly
        self.slide_transmission_ratio = 0.013
        self.right_slide_id = 2
        self.left_slide_id = 7

        # Ankle motor IDs
        self.right_inner_ankle_id = 4
        self.right_outer_ankle_id = 3
        self.left_inner_ankle_id = 8
        self.left_outer_ankle_id = 9

        # Timer
        self.previous_time = self.get_clock().now()
        self.previous_joint_states = None

    
    def position_goal_callback(self, msg):

        # Some joints do not need to be adjusted, so we start from the original message
        # TODO: I don't really like this, I should probably fix
        motor_position_goals = msg.data

        # Apply the slide transmission ratio to the position goal
        # TODO: I think this is the wrong way around
        motor_position_goals[self.right_slide_id] /= -self.slide_transmission_ratio
        motor_position_goals[self.left_slide_id] /= self.slide_transmission_ratio

        right_ankle_goals = ankle.InverseKinematics.ik(motor_position_goals[3], motor_position_goals[4])
        left_ankle_goals = ankle.InverseKinematics.ik(motor_position_goals[8], motor_position_goals[9])
        
        motor_position_goals[self.right_outer_ankle_id] = -right_ankle_goals[0]
        motor_position_goals[self.right_inner_ankle_id] = right_ankle_goals[1]

        motor_position_goals[self.left_inner_ankle_id] = -left_ankle_goals[0]
        motor_position_goals[self.left_outer_ankle_id] = -left_ankle_goals[1]

        self.publish_position_goals(motor_position_goals)

    def torque_goal_callback(self, msg):
        
        # Some joints do not need to be adjusted, so we start from the original message
        # TODO: I don't really like this, I should probably fix
        motor_torque_goals = msg.data

        # Apply the slide transmission ratio to the torque goal
        motor_torque_goals[self.right_slide_id] *= -self.slide_transmission_ratio
        motor_torque_goals[self.left_slide_id] *= self.slide_transmission_ratio

        self.publish_torque_goals(motor_torque_goals)

    def clamp(value, min_val, max_val):
        return max(min_val, min(value, max_val))

    # When we recive a motor state message, translate it into a joint state message
    def motor_state_callback(self, msg):
        # Calculate dt
        current_time = self.get_clock().now()
        dt = current_time - self.previous_time
        ns_to_s = 1 / 1e+9
        dt_sec = dt.nanoseconds * ns_to_s
        self.previous_time = current_time

        joint_states = msg

        # ======= Transform slide from motor to joint space
        joint_states.position[self.left_slide_id] *= self.slide_transmission_ratio
        joint_states.velocity[self.left_slide_id] *= self.slide_transmission_ratio
        joint_states.effort[self.left_slide_id] /= self.slide_transmission_ratio

        joint_states.position[self.right_slide_id] *= -self.slide_transmission_ratio
        joint_states.velocity[self.right_slide_id] *= -self.slide_transmission_ratio
        joint_states.effort[self.right_slide_id] /= -self.slide_transmission_ratio
        
        # ======= Transform ankle from motor to joint space
        joint_states.name[3] = "Right_Foot_Roll"
        joint_states.name[4] = "Right_Foot_Pitch"
        joint_states.name[8] = "Left_Foot_Roll"
        joint_states.name[9] = "Left_Foot_Pitch"

        # Perform forward kinematics to find joint position given motor position
        zero_offset = 0.32
        right_foot_position = ankle.ForwardKinematics.fk(
            joint_states.position[self.right_outer_ankle_id] + zero_offset,
            -joint_states.position[self.right_inner_ankle_id] + zero_offset)

        # Set position
        joint_states.position[3] = self.clamp(right_foot_position[0], -0.2, 0.2)
        joint_states.position[4] = self.clamp(-right_foot_position[1], -0.2, 0.2)

        left_foot_position = ankle.ForwardKinematics.fk(
            joint_states.position[self.left_inner_ankle_id] + zero_offset,
            -joint_states.position[self.left_outer_ankle_id] + zero_offset)

        joint_states.position[8] = self.clamp(left_foot_position[0], -0.2, 0.2)
        joint_states.position[9] = self.clamp(left_foot_position[1], -0.2, 0.2)


        if(self.previous_joint_states):

            # Set velocity
            # TODO: Filter
            joint_states.velocity[3] = (right_foot_position[0] - self.previous_joint_states.position[3]) / dt_sec
            joint_states.velocity[4] = (-right_foot_position[1] - self.previous_joint_states.position[4]) / dt_sec

            # Set velocity 
            # TODO: Filter
            joint_states.velocity[8] = (left_foot_position[0] - self.previous_joint_states.position[8]) / dt_sec
            joint_states.velocity[9] = (left_foot_position[1] - self.previous_joint_states.position[9]) / dt_sec

            self.previous_joint_states = joint_states

        else:
            self.previous_joint_states = joint_states

        # Perform forward dynamics to find joint velocity given motor velocity

        # Perform forward dynamics to find joint torques given motor torques

        self.joint_state_publisher.publish(joint_states)

    # Publish position goals
    def publish_position_goals(self, values):
        msg = Float32MultiArray()
        msg.data = values
        self.position_publisher.publish(msg)
        self.get_logger().info("Published position goals")

    # Publish torque goals
    def publish_torque_goals(self, values):
        msg = Float32MultiArray()
        msg.data = values
        self.torque_publisher.publish(msg)
        self.get_logger().info("Published torque goals")


def main(args=None):
    rclpy.init(args=args)

    slider_joint_controller = SliderJointController()

    rclpy.spin(slider_joint_controller)

    slider_joint_controller.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()