cmake_minimum_required(VERSION 3.8)
project(vesc_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(vesc_driver REQUIRED)
find_package(vesc_hardware_msgs REQUIRED)

## COMPILE
add_library(
  vesc_hardware
  SHARED
  src/vesc_hardware.cpp
)
target_include_directories(vesc_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(vesc_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  vesc_driver::vesc_driver
  ${vesc_hardware_msgs_TARGETS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface vesc_hardware.xml)

# INSTALL
install(
  DIRECTORY include
  DESTINATION include
)
install(TARGETS vesc_hardware
  EXPORT export_vesc_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_vesc_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  vesc_driver
  vesc_hardware_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
