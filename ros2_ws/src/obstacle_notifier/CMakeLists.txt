cmake_minimum_required(VERSION 3.8)
project(obstacle_notifier)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create shared library
add_library(obstacle_notifier_component SHARED
  src/obstalce_notifier_component.cpp
)

ament_target_dependencies(obstacle_notifier_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

#  Register the component
rclcpp_components_register_nodes(obstacle_notifier_component
  "obstacle_notifier::ObstacleNotifierComponent"
)

#  Install the shared library
install(TARGETS obstacle_notifier_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Optional: install include files if any
install(DIRECTORY include/
  DESTINATION include
)

# Required to register the plugin file
ament_package()
