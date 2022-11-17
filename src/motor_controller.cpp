#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "slider_joint_control/MotorManager.h"
#include "slider_joint_control/TMotor.h"
#include "slider_joint_control/TMotorAK80_80.h"
#include "slider_joint_control/TMotorAK60_6.h"
#include "slider_joint_control/TMotorAK10_9.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

enum ControlMode {position, torque, disabled};

class MotorController : public rclcpp::Node
{
  public: // ============================ PUBLIC
  MotorController() : Node("joint_controller")
  {
    // Log that we're starting
    RCLCPP_INFO(this->get_logger(), "Starting SLIDER joint control");


    // Make position subscriber
    position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>
    ("slider/control/joint_position_goals", 10, std::bind(&MotorController::position_callback, this, _1));

    // Make torque subscriber
    torque_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>
    ("slider/control/joint_torque_goals", 10, std::bind(&MotorController::torque_callback, this, _1));

    // Make Kp subscriber
    // Make Kd subscriber

    // control_loop_timer->cancel
    // Init variables
    control_mode = disabled;

    // Sets up the motors
    setup_motors();

    // Home the robot
    connect_and_enable();

    // Start the control loop timer
    // control_loop_timer = this->create_wall_timer(10ms, std::bind(&JointController::motor_control_loop, this));
  }

  void disable_all()
  {
    right_leg_motors.disable_all();
    left_leg_motors.disable_all();
  }

  private: // ========================== PRIVATE

  void setup_motors()
  {
    // Adds all joints to the manager
    right_leg_motors.add_motor(&right_roll);
    right_leg_motors.add_motor(&right_pitch);
    right_leg_motors.add_motor(&right_slide);
    right_leg_motors.add_motor(&right_inner_ankle);
    right_leg_motors.add_motor(&right_outer_ankle);

    left_leg_motors.add_motor(&left_roll);
    left_leg_motors.add_motor(&left_pitch);
    left_leg_motors.add_motor(&left_slide);
    left_leg_motors.add_motor(&left_inner_ankle);
    left_leg_motors.add_motor(&left_outer_ankle);

    // Print out motor names
    right_leg_motors.print_all_motors();
    left_leg_motors.print_all_motors();
  }


  void connect_and_enable()
  {
    // Do the homing thing
    RCLCPP_INFO(this->get_logger(), "Connecting to right motors");
    right_leg_motors.connect();
    RCLCPP_INFO(this->get_logger(), "Enabling right motors");
    right_leg_motors.enable_all();

    RCLCPP_INFO(this->get_logger(), "Connecting to left motors");
    left_leg_motors.connect();
    RCLCPP_INFO(this->get_logger(), "Enabling to left motors");
    left_leg_motors.enable_all();
  }

  // Position control callback
  void position_callback(const std_msgs::msg::Float32MultiArray::ConstPtr& msg)
  {

    // Check if we've recived the correct number of values (10)
    // scream if not
    if(msg->data.size() != 10){
      RCLCPP_ERROR(this->get_logger(), "Incorrect array length published");
      return;
    }

    // Update position goals
    // this is done async so we can send them out at a constant rate, not tied to the sender
    // introduces a time delay, but improves robustness
    position_goals = msg->data;

    // Put robot in position control mode
    control_mode = position;

    // TODO: Add some logic to check that we're not going to do something stupid

    // Update last message recive time
    last_msg_time = std::chrono::steady_clock::now();

    // Print out recived values
    std::stringstream ss;
    ss << "Recived position targets: ";
    std::copy(position_goals.begin(), position_goals.end(), std::ostream_iterator<float>(ss, " "));
    ss << std::endl;
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());


    // End
  }


  // Torque control callback
  void torque_callback(const std_msgs::msg::Float32MultiArray::ConstPtr& msg)
  { 

    // Check if we've recived the correct number of values (10)
    // scream if not
    if(msg->data.size() != 10){
      RCLCPP_ERROR(this->get_logger(), "Incorrect array length published");
      return;
    }

    // Update torque goals
    // this is done async so we can send them out at a constant rate, not tied to the sender
    // introduces a time delay, but improves robustness
    torque_goals = msg->data;

    // Put the robot in torque contorl mode
    control_mode = torque;

    // Update last message recive time
    last_msg_time = std::chrono::steady_clock::now();

    // Print out recived values
    std::stringstream ss;
    ss << "Recived position targets: ";
    std::copy(position_goals.begin(), position_goals.end(), std::ostream_iterator<float>(ss, " "));
    ss << std::endl;
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  }

  // Motor control loop
  void motor_control_loop()
  {

    // Debug prints
    //TODO: add this as a verbose arg
    std::stringstream ss;
    ss << "Current position targets: ";
    std::copy(position_goals.begin(), position_goals.end(), std::ostream_iterator<float>(ss, " "));
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());


    // ==== CONTROL MODES
    if(control_mode == disabled)
    {
      // Send disable
    }else if(control_mode == position)
    {
      // Send position goals
    }else if(control_mode == torque)
    {
      // Send torque goals
    }



    // ===== WATCHDOG

    // Get current time
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    // Print out time delta
    auto time_delta = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_msg_time).count();

    //TODO: put this on verbose arg
    RCLCPP_INFO(this->get_logger(), "Watchdog %li", time_delta);

    // Check if we have violated our watchdog timer and disable motors
    if(time_delta > control_timout_ms)
    {
      RCLCPP_WARN(this->get_logger(), "Control message watchdog timer has run out, disabling motors!");

      // Disable the robot
      control_mode = disabled;
    }
  }


  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr torque_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr Kp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr Kd_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_loop_timer;

  // TODO: Clean
  ControlMode control_mode;
  std::vector<float> position_goals;
  std::vector<float> torque_goals;

  // Watchdog variables
  std::chrono::steady_clock::time_point last_msg_time;
  int64_t control_timout_ms = 500; // Time to wait to recive a control input before we disable motors

  // Set up motors 
  TMotorAK80_80 right_roll = TMotorAK80_80("Right_Roll", 1);
  TMotorAK10_9 right_pitch = TMotorAK10_9("Right_Pitch", 2);
  TMotorAK60_6 right_slide = TMotorAK60_6("Right_Slide", 3);
  TMotorAK60_6 right_inner_ankle = TMotorAK60_6("Right_Foot_Inner", 50);
  TMotorAK60_6 right_outer_ankle = TMotorAK60_6("Right_Foot_Outer", 51);

  TMotorAK80_80 left_roll = TMotorAK80_80("Left_Roll", 11);
  TMotorAK10_9 left_pitch = TMotorAK10_9("Left_Pitch", 10);
  TMotorAK60_6 left_slide = TMotorAK60_6("Left_Slide", 13);
  TMotorAK60_6 left_inner_ankle = TMotorAK60_6("Left_Foot_Inner", 40);
  TMotorAK60_6 left_outer_ankle = TMotorAK60_6("Left_Foot_Outer", 41);

  // Set up managers
  MotorManager right_leg_motors = MotorManager("can0");
  MotorManager left_leg_motors = MotorManager("can1");
};


// Init and start node
int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);

  // Make node and spin
  auto motor_controller = std::make_shared<MotorController>();
  rclcpp::spin(motor_controller);

  // Disable motors and shut down
  motor_controller.get()->disable_all();
  rclcpp::shutdown();

  return 0;
}