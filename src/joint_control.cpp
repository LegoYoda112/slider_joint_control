#include "slider_joint_control/ankleKinematics.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;


class JointController : public rclcpp::Node
{
    public:

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr position_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr torque_publisher;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher;

    JointController() : Node("joint_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Starting SLIDER joint control");

        joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            10
        )
    }
}