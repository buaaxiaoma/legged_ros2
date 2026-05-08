cmake_minimum_required(VERSION 3.8)
project(legged_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(urdf REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_go REQUIRED)

# ------------------------------------------------------------
# Legged system interface
# ------------------------------------------------------------

add_library(legged_system_interface SHARED src/legged_system_interface.cpp)
target_compile_features(legged_system_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(legged_system_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  legged_system_interface
  "rclcpp"
  "hardware_interface"
  "pluginlib"
  "rclcpp_lifecycle"
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(legged_system_interface PRIVATE "LEGGED_ROS2_CONTROL_BUILDING_LIBRARY")


# ------------------------------------------------------------
# For unitree go2
# ------------------------------------------------------------
add_subdirectory(src/robots/unitree_go2)

# ------------------------------------------------------------
# For unitree g1
# ------------------------------------------------------------
add_subdirectory(src/robots/unitree_g1)


# ------------------------------------------------------------
# Export hardware plugins
# ------------------------------------------------------------
pluginlib_export_plugin_description_file(legged_ros2_control legged_ros2_control_plugins.xml)


# ------------------------------------------------------------
# Goal-to-velocity command helper
# ------------------------------------------------------------

add_executable(goal_to_cmd_vel
  src/goal_to_cmd_vel_node.cpp
)
target_compile_features(goal_to_cmd_vel PUBLIC c_std_99 cxx_std_17)
target_include_directories(goal_to_cmd_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  goal_to_cmd_vel
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  unitree_go
)

# ------------------------------------------------------------
# Installation
# ------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS legged_system_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS goal_to_cmd_vel
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories(
  include
)

ament_package()
