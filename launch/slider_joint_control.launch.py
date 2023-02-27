from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    motor_controller_node = Node(
        package='slider_joint_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )

    joint_controller_node = Node(
        package='slider_joint_control',
        executable='slider_joint_controller.py',
        name='joint_controller',
        output='screen'
    )

    return LaunchDescription([
        motor_controller_node,
        joint_controller_node
    ])