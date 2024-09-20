cmake_minimum_required(VERSION 3.8)
project(s300_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Add the library for serial communication
add_library(serialcomm_s300 SHARED
  src/serialcomm_s300.cpp
)
target_include_directories(serialcomm_s300 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(serialcomm_s300
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
)

# Add the executable for Sick S300 driver
add_executable(sick300_driver
  src/sicks300.cpp
)
target_link_libraries(sick300_driver
  serialcomm_s300
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

# Install targets
install(TARGETS
  serialcomm_s300
  sick300_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()