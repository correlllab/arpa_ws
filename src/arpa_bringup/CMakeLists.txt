cmake_minimum_required(VERSION 3.8)
project(arpa_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()