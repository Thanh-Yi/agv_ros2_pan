cmake_minimum_required(VERSION 3.8)
project(wt901_ros2)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Find dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(wt901_driver REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# =========================
# Build node
# =========================
add_executable(wt901_node
  src/wt901_node.cpp
)

# Link C driver library
target_link_libraries(wt901_node
  wt901_driver::wt901_driver
)

# ROS dependencies for node
ament_target_dependencies(
  wt901_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# =========================
# Install
# =========================
install(TARGETS wt901_node
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Testing (optional)
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
