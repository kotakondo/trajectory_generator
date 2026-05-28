cmake_minimum_required(VERSION 3.8)
project(trajectory_generator)

# C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dynus_interfaces REQUIRED)

# ---- Library: trajectory_generator_core ----
add_library(trajectory_generator_core STATIC
  src/trajectories/Circle.cpp
  src/trajectories/Figure8.cpp
  src/trajectories/Line.cpp
)

target_include_directories(trajectory_generator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(trajectory_generator_core
  rclcpp
  geometry_msgs
  nav_msgs
)

# ---- Executable: trajectory_generator_node ----
add_executable(trajectory_generator_node
  src/trajectory_generator_node.cpp
)

target_include_directories(trajectory_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(trajectory_generator_node
  trajectory_generator_core
)

ament_target_dependencies(trajectory_generator_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

# ---- Executable: goal_replay_node ----
add_executable(goal_replay_node
  src/goal_replay_node.cpp
)

target_include_directories(goal_replay_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(goal_replay_node
  rclcpp
  std_srvs
  dynus_interfaces
  nav_msgs
  geometry_msgs
)

# ---- Executable: perfect_tracker_node ----
add_executable(perfect_tracker_node
  src/perfect_tracker_node.cpp
)

ament_target_dependencies(perfect_tracker_node
  rclcpp
  dynus_interfaces
  visualization_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# ---- Install ----
install(TARGETS
  trajectory_generator_node
  goal_replay_node
  perfect_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  trajectories
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
