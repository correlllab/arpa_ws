cmake_minimum_required(VERSION 3.8)
project(arpa_humanoid)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/humanoid_teleport_node.py
  scripts/zero_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
