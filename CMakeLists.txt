cmake_minimum_required(VERSION 3.8)
project(slider_joint_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Python build
find_package(rclpy REQUIRED)


install(DIRECTORY launch urdf rviz meshes
    DESTINATION share/${PROJECT_NAME}
)

# add_executable(motor_controller src/motor_controller.cpp src/MotorManager.cpp src/TMotor.cpp src/canbus.cpp)
# add_executable(ankle_kinematics src/ankleKinematics.cpp)
add_executable(joint_control src/joint_control.cpp src/ankleKinematics.cpp)


# target_include_directories(ankle_kinematics PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(ankle_kinematics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(joint_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joint_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(joint_control rclcpp std_msgs sensor_msgs)

# target_include_directories(motor_controller PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(motor_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(motor_controller rclcpp std_msgs sensor_msgs)

# target_include_directories(test_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(test_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(test_node rclcpp std_msgs)


# ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY scripts/ankle
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_pub.py 
  scripts/slider_joint_controller.py 
  scripts/system_id.py 
  scripts/can_setup.sh
  scripts/trajectory_playback.py
  DESTINATION lib/${PROJECT_NAME} 
)

# install(TARGETS motor_controller
#   DESTINATION lib/${PROJECT_NAME})

install(TARGETS joint_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()