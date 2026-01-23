cmake_minimum_required(VERSION 3.8)
project(enemy_tracking)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(nav2_util REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

add_executable(enemy_tracking_node src/enemy_tracking_node.cpp)
ament_target_dependencies(enemy_tracking_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp_action
  visualization_msgs
  rm_interfaces
        nav2_util
        behaviortree_ros2

)

install(TARGETS enemy_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()