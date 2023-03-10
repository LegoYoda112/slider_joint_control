#include "slider_joint_control/ankleKinematics.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointController : public rclcpp::Node
{   
    private:
        const float SLIDE_TRANSMISSION_RATIO = 0.013;
        const int RIGHT_SLIDE_ID = 2;
        const int LEFT_SLIDE_ID = 7;

        const int RIGHT_INNER_ANKLE_ID = 3;
        const int RIGHT_OUTER_ANKLE_ID = 4;
        const int LEFT_INNER_ANKLE_ID = 8;
        const int LEFT_OUTER_ANKLE_ID = 9;

    
    public:


    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_position_pub_;


    JointController() : Node("joint_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Starting SLIDER joint control");

        motor_position_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("slider/control/motor/position_goals", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        
        // Make position subscriber
        joint_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>
        ("slider/control/joint/position_goals", 10, std::bind(&JointController::position_goal_callback, this, _1));
        
        // Make motor state subscriber
        motor_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>
        ("slider/control/motor/states", 10, std::bind(&JointController::motor_state_callback, this, _1));
        

        
    }
    

    // Converts joint state goals to motor state goals and publish
    void position_goal_callback(const std_msgs::msg::Float32MultiArray::ConstPtr& msg)
    {
        // Check if we've recived the correct number of values (10)
        // scream if not
        if(msg->data.size() != 10){
            RCLCPP_ERROR(this->get_logger(), "Incorrect array length published");
            return;
        }

        float motor_targets[10];

        // Hip roll and pitch
        motor_targets[0] = msg.data[0];
        motor_targets[1] = msg.data[1];

        motor_targets[5] = msg.data[5];
        motor_targets[6] = msg.data[6];

        // Slide
        motor_targets[RIGHT_SLIDE_ID] = msg.data[RIGHT_SLIDE_ID] / SLIDE_TRANSMISSION_RATIO;
        motor_targets[LEFT_SLIDE_ID] = msg.data[LEFT_SLIDE_ID] / SLIDE_TRANSMISSION_RATIO;
        
        // Right ankle inverse kinematics
        float motor_left;
        float motor_right;
        ankleIK(msg.data[3], msg.data[4], motor_left, motor_right);
        motor_targets[RIGHT_INNER_ANKLE_ID] = motor_left;
        motor_targets[RIGHT_OUTER_ANKLE_ID] = motor_right;

        // Left ankle
        ankleIK(msg.data[8], msg.data[9], motor_left, motor_right);
        motor_targets[LEFT_OUTER_ANKLE_ID] = motor_left;
        motor_targets[LEFT_INNER_ANKLE_ID] = motor_right;
        
        std_msgs::msg::Float32MultiArray output = output_msg;
        output_msg.data = motor_targets;

        motor_position_pub_->publish(output_msg);
    }
}