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


# Linear Actuator Hardware Interface Library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/parker_controller_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(${PROJECT_NAME} parker_core)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY urdf config srdf launch
  DESTINATION share/${PROJECT_NAME}
)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface parker_controller_interface.xml)


ament_package()
