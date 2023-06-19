#include "slider_joint_control/ankleKinematics.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;


class JointController : public rclcpp::Node
{   
    private:
        const float SLIDE_TRANSMISSION_RATIO = 0.013;

        const int RIGHT_ROLL_ID = 0;
        const int RIGHT_PITCH_ID = 1;
        const int RIGHT_SLIDE_ID = 2;
        const int RIGHT_INNER_ANKLE_ID = 3;
        const int RIGHT_OUTER_ANKLE_ID = 4;

        const int RIGHT_ANKLE_ROLL_ID = 3;
        const int RIGHT_ANKLE_PITCH_ID = 4;

        const int LEFT_ROLL_ID = 5;
        const int LEFT_PITCH_ID = 6;
        const int LEFT_SLIDE_ID = 7;
        const int LEFT_INNER_ANKLE_ID = 8;
        const int LEFT_OUTER_ANKLE_ID = 9;

        const int LEFT_ANKLE_ROLL_ID = 8;
        const int LEFT_ANKLE_PITCH_ID = 9;

    
    public:


    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_position_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_position_sub_;  
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motor_state_sub_;


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
        
        float motor_1;
        float motor_2;
        ankleKinematics::ankleIK(0.0, 0.0, motor_1, motor_2);

        cout << motor_1 << endl;
        cout << motor_2 << endl;
        
        float alpha;
        float beta;

        ankleKinematics::ankleFK(0.18, 0.18, alpha, beta);

        cout << alpha << endl;
        cout << beta << endl;
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
        
        RCLCPP_INFO(this->get_logger(), "Recived");

        std_msgs::msg::Float32MultiArray output_msg;
        output_msg.data.resize(10);

        // Hip roll and pitch
        output_msg.data[RIGHT_ROLL_ID] = msg->data[RIGHT_ROLL_ID];
        output_msg.data[RIGHT_PITCH_ID] = msg->data[RIGHT_PITCH_ID];

        output_msg.data[LEFT_ROLL_ID] = msg->data[LEFT_ROLL_ID];
        output_msg.data[RIGHT_ROLL_ID] = msg->data[RIGHT_ROLL_ID];

        // Slide
        output_msg.data[RIGHT_SLIDE_ID] = msg->data[RIGHT_SLIDE_ID] / SLIDE_TRANSMISSION_RATIO;
        output_msg.data[LEFT_SLIDE_ID] = msg->data[LEFT_SLIDE_ID] / SLIDE_TRANSMISSION_RATIO;
        
        // Right ankle inverse kinematics
        float motor_left;
        float motor_right;

        ankleKinematics::ankleIK(-msg->data[RIGHT_ANKLE_ROLL_ID], -msg->data[RIGHT_ANKLE_PITCH_ID], motor_right, motor_left);
        output_msg.data[RIGHT_INNER_ANKLE_ID] = motor_left;
        output_msg.data[RIGHT_OUTER_ANKLE_ID] = motor_right;

        // Left ankle
        ankleKinematics::ankleIK(-msg->data[LEFT_ANKLE_ROLL_ID], msg->data[LEFT_ANKLE_PITCH_ID], motor_right, motor_left);
        output_msg.data[LEFT_OUTER_ANKLE_ID] = motor_left;
        output_msg.data[LEFT_INNER_ANKLE_ID] = motor_right;

        motor_position_pub_->publish(output_msg);
    }

    void motor_state_callback (const sensor_msgs::msg::JointState::ConstPtr& motor_state)
    {
        sensor_msgs::msg::JointState joint_state;

        // Resize arrays
        joint_state.name.resize(10);
        joint_state.position.resize(10);
        joint_state.velocity.resize(10);
        joint_state.effort.resize(10);


        // Right Leg
        joint_state.name[RIGHT_ROLL_ID] = "Right_Roll";
        joint_state.position[RIGHT_ROLL_ID] = motor_state->position[RIGHT_ROLL_ID];
        joint_state.velocity[RIGHT_ROLL_ID] = motor_state->velocity[RIGHT_ROLL_ID];

        joint_state.name[RIGHT_PITCH_ID] = "Right_Pitch";
        joint_state.position[RIGHT_PITCH_ID] = motor_state->position[RIGHT_PITCH_ID];
        joint_state.velocity[RIGHT_PITCH_ID] = motor_state->velocity[RIGHT_PITCH_ID];

        // NOTE ADDED A NEGATIVE
        joint_state.name[RIGHT_SLIDE_ID] = "Right_Slide";
        joint_state.position[RIGHT_SLIDE_ID] = -motor_state->position[RIGHT_SLIDE_ID] * SLIDE_TRANSMISSION_RATIO;
        joint_state.velocity[RIGHT_SLIDE_ID] = -motor_state->velocity[RIGHT_SLIDE_ID] * SLIDE_TRANSMISSION_RATIO;

        float right_foot_roll;
        float right_foot_pitch;

        float right_foot_roll_vel;
        float right_foot_pitch_vel;

        float right_inner_position = motor_state->position[RIGHT_INNER_ANKLE_ID];
        float right_inner_velocity = motor_state->velocity[RIGHT_INNER_ANKLE_ID];
        float right_outer_position = motor_state->position[RIGHT_OUTER_ANKLE_ID];
        float right_outer_velocity = motor_state->velocity[RIGHT_OUTER_ANKLE_ID];

        // Calculate foot roll and pitch given motor angles
        ankleKinematics::ankleFK(right_outer_position, right_inner_position, right_foot_roll, right_foot_pitch);
        // Calculate foot roll and pitch velocity given motor velocities
        ankleKinematics::ankleFKvel(right_outer_position, right_inner_position, right_outer_velocity, right_inner_velocity, right_foot_pitch_vel, right_foot_roll_vel, 0.05);

        joint_state.name[RIGHT_ANKLE_ROLL_ID] = "Right_Foot_Roll";
        joint_state.position[RIGHT_ANKLE_ROLL_ID] = -right_foot_roll;
        joint_state.velocity[RIGHT_ANKLE_ROLL_ID] = -right_foot_roll_vel;

        joint_state.name[RIGHT_ANKLE_PITCH_ID] = "Right_Foot_Pitch";
        joint_state.position[RIGHT_ANKLE_PITCH_ID] = -right_foot_pitch;
        joint_state.velocity[RIGHT_ANKLE_PITCH_ID] = -right_foot_pitch_vel;

        // Left Leg
        joint_state.name[LEFT_ROLL_ID] = "Left_Roll";
        joint_state.position[LEFT_ROLL_ID] = motor_state->position[LEFT_ROLL_ID];
        joint_state.velocity[LEFT_ROLL_ID] = motor_state->velocity[LEFT_ROLL_ID];

        joint_state.name[LEFT_PITCH_ID] = "Left_Pitch";
        joint_state.position[LEFT_PITCH_ID] = motor_state->position[LEFT_PITCH_ID];
        joint_state.velocity[LEFT_PITCH_ID] = motor_state->velocity[LEFT_PITCH_ID];

        joint_state.name[LEFT_SLIDE_ID] = "Left_Slide";
        joint_state.position[LEFT_SLIDE_ID] = motor_state->position[LEFT_SLIDE_ID] * SLIDE_TRANSMISSION_RATIO;
        joint_state.velocity[LEFT_SLIDE_ID] = motor_state->velocity[LEFT_SLIDE_ID] * SLIDE_TRANSMISSION_RATIO;


        float left_foot_roll;
        float left_foot_pitch;

        float left_foot_roll_vel;
        float left_foot_pitch_vel;

        float left_inner_position = motor_state->position[LEFT_INNER_ANKLE_ID];
        float left_inner_velocity = motor_state->velocity[LEFT_INNER_ANKLE_ID];
        float left_outer_position = motor_state->position[LEFT_OUTER_ANKLE_ID];
        float left_outer_velocity = motor_state->velocity[LEFT_OUTER_ANKLE_ID];

        // Calculate foot roll and pitch given motor angles
        ankleKinematics::ankleFK(left_outer_position, left_inner_position, left_foot_roll, left_foot_pitch);
        // Calculate foot roll and pitch velocity given motor velocities
        ankleKinematics::ankleFKvel(left_outer_position, left_inner_position, left_outer_velocity, left_inner_velocity, left_foot_pitch_vel, left_foot_roll_vel, 0.05);

        joint_state.name[LEFT_ANKLE_ROLL_ID] = "Left_Foot_Roll";
        joint_state.position[LEFT_ANKLE_ROLL_ID] = left_foot_roll;
        joint_state.velocity[LEFT_ANKLE_ROLL_ID] = left_foot_roll_vel;

        joint_state.name[LEFT_ANKLE_PITCH_ID] = "Left_Foot_Pitch";
        joint_state.position[LEFT_ANKLE_PITCH_ID] = left_foot_pitch;
        joint_state.velocity[LEFT_ANKLE_PITCH_ID] = left_foot_pitch_vel;

        joint_state_pub_->publish(joint_state);
    }
};

// Init and start node
int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);

  // Make node and spin
  auto joint_controller = std::make_shared<JointController>();

  rclcpp::spin(joint_controller);

  rclcpp::shutdown();

  return 0;
}