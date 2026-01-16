cmake_minimum_required(VERSION 3.5)
project(arpa_gui)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(ur_manipulation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(arpa_gui
  src/main.cpp
  src/pose_window.cpp
  include/arpa_gui/pose_window.hpp
)

target_link_libraries(arpa_gui
  Qt5::Widgets
)

ament_target_dependencies(arpa_gui
  rclcpp
  tf2_ros
  tf2
  ur_manipulation
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
)

target_include_directories(arpa_gui PUBLIC
  include
)

install(TARGETS arpa_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()