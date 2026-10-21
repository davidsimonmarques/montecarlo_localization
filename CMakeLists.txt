cmake_minimum_required(VERSION 3.8)
project(montecarlo_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Core MCL library
add_library(montecarlo_localization_core SHARED
  src/map_model.cpp
  src/motion_model.cpp
  src/sensor_model.cpp
  src/resampling.cpp
  src/mcl_engine.cpp
  src/amcl_engine.cpp
  src/mhmcl_engine.cpp
  src/mhamcl_engine.cpp
  src/mcl_node.cpp
  src/amcl_node.cpp
  src/mhmcl_node.cpp
  src/mhamcl_node.cpp
)

target_include_directories(montecarlo_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(montecarlo_localization_core
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Executable Node: Classical MCL
add_executable(montecarlo_localization_node
  src/main.cpp
)

target_link_libraries(montecarlo_localization_node
  montecarlo_localization_core
)

ament_target_dependencies(montecarlo_localization_node
  rclcpp
)

# Executable Node: Adaptive MCL (AMCL)
add_executable(amcl_node
  src/amcl_main.cpp
)

target_link_libraries(amcl_node
  montecarlo_localization_core
)

ament_target_dependencies(amcl_node
  rclcpp
)

# Executable Node: Metropolis-Hastings MCL (MH-MCL)
add_executable(mhmcl_node
  src/mhmcl_main.cpp
)

target_link_libraries(mhmcl_node
  montecarlo_localization_core
)

ament_target_dependencies(mhmcl_node
  rclcpp
)

# Executable Node: Metropolis-Hastings AMCL (MH-AMCL)
add_executable(mhamcl_node
  src/mhamcl_main.cpp
)

target_link_libraries(mhamcl_node
  montecarlo_localization_core
)

ament_target_dependencies(mhamcl_node
  rclcpp
)

# Goal Relay Node (bridges RViz /goal_pose to /goal_position)
add_executable(goal_relay_node
  src/goal_relay.cpp
)

ament_target_dependencies(goal_relay_node
  rclcpp
  geometry_msgs
)

# Install targets and directories
install(
  TARGETS
    montecarlo_localization_core
    montecarlo_localization_node
    amcl_node
    mhmcl_node
    mhamcl_node
    goal_relay_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(montecarlo_localization_core)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
