cmake_minimum_required(VERSION 3.8)
project(arpa_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
