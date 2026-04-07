cmake_minimum_required(VERSION 3.8)
project(arpa_helper_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/record_poses.py
  scripts/remove_screws_from_recorded_poses.py
  scripts/remove_screws_from_vision.py
  scripts/core_functionality_node.py
  scripts/scan_battery_action_server.py
  scripts/scan_battery.py
  scripts/run_benchmark.py
  scripts/plot_benchmark.py
  scripts/vla_data_collection_node.py
  scripts/parts_visualizer_node.py
  scripts/battery_pointcloud_publisher.py
  scripts/sim_remove_sequence_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource files (parts lists, schemas)
install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
