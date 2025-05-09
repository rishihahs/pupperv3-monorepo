cmake_minimum_required(VERSION 3.5)
project(dancing)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Add the executable
add_executable(velocity_recorder src/velocity_recorder.cpp)
target_include_directories(velocity_recorder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(velocity_replayer src/velocity_replayer.cpp)
target_include_directories(velocity_replayer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link against ROS 2 libraries
ament_target_dependencies(velocity_recorder
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  controller_manager_msgs
)

ament_target_dependencies(velocity_replayer
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  controller_manager_msgs
)

# Link with pthread and rt libraries
target_link_libraries(velocity_replayer
  pthread
  rt
)

# Install the executable
install(TARGETS velocity_recorder velocity_replayer
  DESTINATION lib/${PROJECT_NAME}
)

# Export package
ament_package()
