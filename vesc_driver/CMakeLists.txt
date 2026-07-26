cmake_minimum_required(VERSION 3.5)
project(vesc_driver)

# Set minimum C++ standard to C++14
if(NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++14")
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

###########
## Build ##
###########

# Core library (no ROS dependencies)
add_library(${PROJECT_NAME}_lib SHARED
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
  src/vesc_device_uuid_lookup.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_lib
  ${CMAKE_THREAD_LIBS_INIT}
)
ament_target_dependencies(${PROJECT_NAME}_lib
  serial_driver
)

# ROS node library
add_library(${PROJECT_NAME} SHARED
  src/vesc_driver.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  vesc_msgs
  sensor_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN vesc_driver::VescDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

# Device namer executable
add_executable(vesc_device_namer
  src/vesc_device_namer.cpp
)
target_link_libraries(vesc_device_namer
  ${PROJECT_NAME}_lib
)

#############
## Install ##
#############

# Install headers
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install libraries
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS vesc_device_namer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

#############
## Exports ##
#############

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  vesc_msgs
  serial_driver
  sensor_msgs
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
