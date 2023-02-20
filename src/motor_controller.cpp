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

enum ControlMode {position, torque, disabled, error};

class MotorController : public rclcpp::Node
{
  public: // ============================ PUBLIC
  MotorController() : Node("joint_controller")
  {
    // Log that we're starting
    RCLCPP_INFO(this->get_logger(), "Starting SLIDER joint control");


    // Make position subscriber
    position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>
    ("slider/control/motor/position_goals", 10, std::bind(&MotorController::position_callback, this, _1));

    // Make torque subscriber
    torque_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>
    ("slider/control/motor/torque_goals", 10, std::bind(&MotorController::torque_callback, this, _1));

    leg_motor_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("slider/control/motor/states", 10);

    // Make Kp subscriber
    // Make Kd subscriber

    // control_loop_timer->cancel
    // Init variables
    control_mode = disabled;

    // Sets up the motors
    setup_motors();

    // Connect to the robot and enable motors
    connect();
    enable();

    // Set zero offsets
    set_zero_offsets();

    // Home
    // home();

    // Set constants
    set_constants();

    // Start the control loop timer
    control_loop_timer = this->create_wall_timer(10ms, std::bind(&MotorController::motor_control_loop, this));
  }


  // Start motor state read threads
  void start_read_threads()
  {
    left_leg_motors.start_read_thread();
    right_leg_motors.start_read_thread();
  }

  // Wait until motor state read threads are finished
  void join_read_threads()
  {
    left_leg_motors.join_read_thread();
    right_leg_motors.join_read_thread();
  }

  void disable_all()
  {
    right_leg_motors.disable_all();
    left_leg_motors.disable_all();
  }

  private: // ========================== PRIVATE

  void setup_motors()
  {
    // Add all motors to the manager
    left_leg_motors.add_motor(&left_roll);
    left_leg_motors.add_motor(&left_pitch);
    left_leg_motors.add_motor(&left_slide);
    left_leg_motors.add_motor(&left_inner_ankle);
    left_leg_motors.add_motor(&left_outer_ankle);

    right_leg_motors.add_motor(&right_roll);
    right_leg_motors.add_motor(&right_pitch);
    right_leg_motors.add_motor(&right_slide);
    right_leg_motors.add_motor(&right_inner_ankle);
    right_leg_motors.add_motor(&right_outer_ankle);


    // Print out motor names
    right_leg_motors.print_all_motors();
    left_leg_motors.print_all_motors();
  }

  void zero_all()
  {
    right_roll.send_zero_encoder();
    right_pitch.send_zero_encoder();
    right_slide.send_zero_encoder();
    right_inner_ankle.send_zero_encoder();
    right_outer_ankle.send_zero_encoder();

    left_roll.send_zero_encoder();
    left_pitch.send_zero_encoder();
    left_slide.send_zero_encoder();
    left_inner_ankle.send_zero_encoder();
    left_outer_ankle.send_zero_encoder();
  }

  // Sets the zero offset of each joint
  void set_zero_offsets()
  {
    right_roll.set_zero_offset(-0.02);
    right_pitch.set_zero_offset(-0.15);
    right_slide.set_zero_offset(-0.48);
    right_inner_ankle.set_zero_offset(0.294);
    right_outer_ankle.set_zero_offset(-0.521);

    left_roll.set_zero_offset(-0.042);
    left_pitch.set_zero_offset(0.338);
    left_slide.set_zero_offset(0.007);
    left_inner_ankle.set_zero_offset(0.077);
    left_outer_ankle.set_zero_offset(-0.232);


    // TODO: Add into motor class
    // Adjust zero offset if we think we've zeroed wrong
    if(left_pitch.position < 0.05){
      float new_zero_offset = left_pitch.get_zero_offset() - 2 * 3.1415 / 9;
      left_pitch.set_zero_offset(new_zero_offset);
    }

    // Adjust zero offset if we think we've zeroed wrong
    if(left_pitch.position > 0.05){
      float new_zero_offset = left_pitch.get_zero_offset() + 2 * 3.1415 / 9;
      left_pitch.set_zero_offset(new_zero_offset);
    }
  }


  void connect()
  {
    // Do the homing thing
    RCLCPP_DEBUG(this->get_logger(), "Connecting to right motors");
    right_leg_motors.connect();

    RCLCPP_DEBUG(this->get_logger(), "Connecting to left motors");
    left_leg_motors.connect();
  }

  void enable()
  {
    // Enable
    RCLCPP_INFO(this->get_logger(), "Enabling right motors");
    right_leg_motors.enable_all();

    RCLCPP_INFO(this->get_logger(), "Enabling to left motors");
    left_leg_motors.enable_all();


  }

  void home()
  {
    left_slide.run_to_home(1.0);
    left_roll.run_to_home(0.5);
    left_pitch.run_to_home(0.5);

    right_slide.run_to_home(1.0);
    right_roll.run_to_home(0.5);
    right_pitch.run_to_home(0.5);
  }

  void set_constants()
  {
    // Set constants
    left_roll.set_constants(30.0, 0.5);
    left_pitch.set_constants(10.0, 0.5);
    left_slide.set_constants(20.0, 0.0);
    left_inner_ankle.set_constants(0.5, 0.2);
    left_outer_ankle.copy_constants(&left_inner_ankle);

    right_roll.copy_constants(&left_roll);
    right_pitch.copy_constants(&left_pitch);
    right_slide.copy_constants(&right_slide);
    right_inner_ankle.copy_constants(&left_inner_ankle);
    right_outer_ankle.copy_constants(&left_inner_ankle);
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

    // Print out recived values
    std::stringstream ss;
    ss << "Recived position targets: ";
    std::copy(msg->data.begin(), msg->data.end(), std::ostream_iterator<float>(ss, " "));
    ss << std::endl;
    RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());

    // Update position goals
    // this is done async so we can send them out at a constant rate, not tied to the sender
    // introduces a time delay, but improves robustness

    sensor_msgs::msg::JointState right_leg_motor_states = right_leg_motors.get_joint_states();
    sensor_msgs::msg::JointState left_leg_motor_states = left_leg_motors.get_joint_states();

    position_goals[0] = right_leg_motor_states.position[0];
    position_goals[1] = right_leg_motor_states.position[1];
    position_goals[2] = right_leg_motor_states.position[2];
    position_goals[3] = right_leg_motor_states.position[3];
    position_goals[4] = right_leg_motor_states.position[4];

    position_goals[5] = left_leg_motor_states.position[0];
    position_goals[6] = left_leg_motor_states.position[1];
    position_goals[7] = left_leg_motor_states.position[2];
    position_goals[8] = left_leg_motor_states.position[3];
    position_goals[9] = left_leg_motor_states.position[4];

    // Check the difference between the current goals and the desired goal. 
    // If the angle difference is too great, throw an error and dissable the robot
    for(int i = 0; i <= 9; i += 1){

      // Adjust for slide
      if(i == 2 || i == 7){
        maximum_position_change_rads = 10.0;
      }

      float target_difference = position_goals[i] - msg->data[i];

      if(abs(target_difference) > maximum_position_change_rads){
        RCLCPP_ERROR(this->get_logger(), "Position change is too large %f %i", target_difference, i);

        control_mode = error;

        return;
      }
    }

    position_goals = msg->data;

    // Put robot in position control mode
    control_mode = position;

    // TODO: Add some logic to check that we're not going to do something stupid

    // Update last message recive time
    last_msg_time = std::chrono::steady_clock::now();


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
    if(control_mode == error){
      RCLCPP_ERROR(this->get_logger(), "Motor controller is in error state");

      // If error, shut down motors
      disable_all();
    }else if(control_mode == disabled)
    {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Sending 0 torque goals");

      // Soft disable
      right_roll.send_torque_goal(0.0);
      right_pitch.send_torque_goal(0.0);
      right_slide.send_torque_goal(0.0);
      right_inner_ankle.send_torque_goal(0.0);
      right_outer_ankle.send_torque_goal(0.0);

      left_roll.send_torque_goal(0.0);
      left_pitch.send_torque_goal(0.0);
      left_slide.send_torque_goal(0.0);
      left_inner_ankle.send_torque_goal(0.0);
      left_outer_ankle.send_torque_goal(0.0);
      
    }else if(control_mode == position)
    { 
      RCLCPP_INFO_STREAM(this->get_logger(), "Sending position goals");

      // set constnats
      // TODO: Figure out why it only works here
      left_roll.set_constants(100.0, 5.0);
      left_pitch.set_constants(100.0, 5.0);
      left_slide.set_constants(5.0, 0.5);
      // left_slide.set_constants(0.0, 0.0);
      left_inner_ankle.set_constants(10.0, 0.3);
      left_outer_ankle.copy_constants(&left_inner_ankle);

      right_roll.copy_constants(&left_roll);
      right_pitch.copy_constants(&left_pitch);
      right_slide.copy_constants(&left_slide);
      right_inner_ankle.copy_constants(&left_inner_ankle);
      right_outer_ankle.copy_constants(&left_inner_ankle);

      // Send position goals
      right_roll.send_position_goal(position_goals[0]);
      right_pitch.send_position_goal(position_goals[1]);
      right_slide.send_position_goal(position_goals[2]);
      // right_slide.send_position_goal(0.0);
      right_inner_ankle.send_position_goal(position_goals[3]);
      right_outer_ankle.send_position_goal(position_goals[4]);

      left_roll.send_position_goal(position_goals[5]);
      left_pitch.send_position_goal(position_goals[6]);
      left_slide.send_position_goal(position_goals[7]);
      // left_slide.send_position_goal(0.0);
      left_inner_ankle.send_position_goal(position_goals[8]);
      left_outer_ankle.send_position_goal(position_goals[9]);

    }else if(control_mode == torque)
    {
      // Send torque goals
    }

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Updating motor states");
    // Publish motor states
    publish_motor_states();

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Checking watchdog");

    // ===== WATCHDOG

    // Get current time
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    // Print out time delta
    auto time_delta = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_msg_time).count();

    // TODO: put this on verbose arg
    RCLCPP_DEBUG(this->get_logger(), "Watchdog %li", time_delta);

    // Check if we have violated our watchdog timer and disable motors
    if(time_delta > control_timout_ms)
    {
      RCLCPP_WARN(this->get_logger(), "Control message watchdog timer has run out, disabling motors!");

      // Disable the robot
      control_mode = disabled;
    }
  }

  void publish_motor_states()
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Making joint state objects");

    // Get joint states
    // Uses locks to prevent reading and writing from the motor states simultaniously
    // when the read is occuring in another thread
    right_leg_motors.aquire_motor_state_lock();
    sensor_msgs::msg::JointState leg_motor_states = right_leg_motors.get_joint_states();
    right_leg_motors.release_motor_state_lock();

    left_leg_motors.aquire_motor_state_lock();
    sensor_msgs::msg::JointState left_leg_joint_states = left_leg_motors.get_joint_states();
    left_leg_motors.release_motor_state_lock();


    RCLCPP_DEBUG_STREAM(this->get_logger(), "Concatining joint states");

    // Concatinate joint efforts
    leg_motor_states.effort.insert(
        leg_motor_states.effort.end(),
        std::make_move_iterator(left_leg_joint_states.effort.begin()),
        std::make_move_iterator(left_leg_joint_states.effort.end())
    );

    // Concatinate joint velocities
    leg_motor_states.velocity.insert(
        leg_motor_states.velocity.end(),
        std::make_move_iterator(left_leg_joint_states.velocity.begin()),
        std::make_move_iterator(left_leg_joint_states.velocity.end())
    );

    // Concatinate joint positions
    leg_motor_states.position.insert(
        leg_motor_states.position.end(),
        std::make_move_iterator(left_leg_joint_states.position.begin()),
        std::make_move_iterator(left_leg_joint_states.position.end())
    );
    
    // Concatinate joint names
    leg_motor_states.name.insert(
        leg_motor_states.name.end(),
        std::make_move_iterator(left_leg_joint_states.name.begin()),
        std::make_move_iterator(left_leg_joint_states.name.end())
    );

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Setting header");
    leg_motor_states.header.stamp = get_clock()->now();

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing");
    leg_motor_state_publisher_->publish(leg_motor_states);

  }


  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr torque_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr Kp_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr Kd_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr leg_motor_state_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_loop_timer;

  // TODO: Clean
  ControlMode control_mode;
  std::vector<float> position_goals{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<float> torque_goals;
  float maximum_position_change_rads = 0.3; // Maximum instant position change

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

  // Set up threads
};


// Init and start node
int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);

  // Make node and spin
  auto motor_controller = std::make_shared<MotorController>();
  motor_controller.get()->start_read_threads();

  rclcpp::spin(motor_controller);

  // Disable motors and shut down
  motor_controller.get()->disable_all();
  motor_controller.get()->join_read_threads();
  rclcpp::shutdown();

  return 0;
}