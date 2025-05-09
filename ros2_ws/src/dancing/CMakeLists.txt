cmake_minimum_required(VERSION 3.5)
project(joy_utils)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service message
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ReplayJoy.srv"
  DEPENDENCIES std_msgs
)

# Add the executable
add_executable(joy_recorder src/joy_recorder.cpp)

target_include_directories(joy_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link against ROS 2 libraries
ament_target_dependencies(joy_recorder
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  controller_manager_msgs
)

rosidl_target_interfaces(joy_recorder
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executable
install(TARGETS joy_recorder
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
