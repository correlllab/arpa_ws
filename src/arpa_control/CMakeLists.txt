cmake_minimum_required(VERSION 3.8)
project(arpa_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanToPose.srv"
  "srv/PlanToJoint.srv"
  "srv/PlanLinearActuator.srv"
  "srv/ExecutePlan.srv"
  "srv/StopMotion.srv"
  "srv/GetPointCloud.srv"
  "srv/FixFailedPlan.srv"
  "srv/GetPoseCostMatrix.srv"
  DEPENDENCIES geometry_msgs sensor_msgs moveit_msgs
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

add_executable(motion_control_node src/motion_control_node.cpp)
ament_target_dependencies(motion_control_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  rosidl_default_runtime
  tf2_ros
  visualization_msgs
)
target_link_libraries(motion_control_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
add_dependencies(motion_control_node 
  ${PROJECT_NAME}__rosidl_generator_cpp
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(TARGETS motion_control_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
