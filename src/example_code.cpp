#include <iostream>
#include <thread>
using namespace std;
 
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// using namespace std::chrono_literals;

// class TestNode : public rclcpp::Node
// {
//     public:
//     TestNode() : Node("test_node")
//     {
//         RCLCPP_INFO(this->get_logger(), "Starting node");

//         loop_timer = this->create_wall_timer(500ms, std::bind(&TestNode::timer_loop, this));
//     }

//     void timer_loop()
//     {
//         RCLCPP_INFO(this->get_logger(), "Loop!");
//     }

//     void loop_print_hello(int test)
//     {   
//         rclcpp::Rate loop_rate(1); // 1 hz
//         while(rclcpp::ok())
//         {
//             std::cout << "test" << std::endl;
//             // RCLCPP_INFO(this->get_logger(), "hello!");
//             loop_rate.sleep();
//             // usleep(500000);
//         }
//     }

//     // void start_hello_thread()
//     // {
//     //     // std::thread test_thread(&TestNode::loop_print_hello, 0);
//     //     return
//     // }

//     rclcpp::TimerBase::SharedPtr loop_timer;
// };

void foo(int Z)
{
    std::cout << "test" << std::endl;
}


int main()
{  
    std::thread th1(foo, 3);
    
    th1.join();

    return 0;
}

