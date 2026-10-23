################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(ffw_vision_msgs)

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Declare ROS messages
################################################################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PixelObservation.msg"
  DEPENDENCIES std_msgs
)

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()
