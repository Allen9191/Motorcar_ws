cmake_minimum_required(VERSION 3.8)
project(joy_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(joy_control src/Modectl.cpp)
ament_target_dependencies(joy_control serial rclcpp sensor_msgs geometry_msgs std_msgs)


# install the executable in the lib folder to make it detectable through setup.bash
install(TARGETS 
  joy_control
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()
