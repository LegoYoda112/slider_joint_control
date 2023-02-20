#include "TMotor.h"

#include "canbus.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <string.h>
#include <vector>
#include <thread>
#include <mutex> 

#ifndef MOTORMANAGER_H
#define MOTORMANAGER_H

class MotorManager
{
    protected:
        int socket_num;
        canbus bus;
        vector<TMotor*> motors;
        thread read_thread;
    
    public:
        MotorManager(std::string name){
            this->bus = canbus(name);
        }

        void connect();

        void add_motor(TMotor* motor);

        void print_all_motors();

        void enable_all();

        void disable_all();

        void home_all();

        void read_one();

        void read_all();

        void read_loop();

        void start_read_thread();

        void join_read_thread();

        void aquire_motor_state_lock();

        void release_motor_state_lock();

        void home_all_individual(float);

        void send_all_zero();

        void soft_stop_hold();

        void soft_stop_dampen();

        sensor_msgs::msg::JointState get_joint_states();

        std::mutex motor_state_lock;
};

#endif