cmake_minimum_required(VERSION 3.8)
project(parker_controller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories for C++
include_directories(include)

# =============================================================================
# C++ Section
# =============================================================================

# ParkerCore library
add_library(parker_core SHARED
  src/parker_core.cpp
)
target_include_directories(parker_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(parker_core rclcpp)

# Install C++ library
install(TARGETS parker_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export library for downstream packages
ament_export_targets(export_parker_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(TARGETS parker_core
  EXPORT export_parker_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# =============================================================================
# ros2_control Hardware Interface
# =============================================================================

# =============================================================================
# Python Section
# =============================================================================

ament_package()
