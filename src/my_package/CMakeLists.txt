cmake_minimum_required(VERSION 3.5)
project(my_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()