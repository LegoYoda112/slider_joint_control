#include "slider_joint_control/ankleKinematics.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointController : public rclcpp::Node
{
    public:
    JointController() : Node("joint_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Starting SLIDER joint control")
    }
}