cmake_minimum_required(VERSION 3.8)
project(r2)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-13")  #

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++23 -fconcepts-diagnostics-depth=3 -Wno-subobject-linkage)  #
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)  #
find_package(std_msgs REQUIRED)  #
find_package(std_srvs REQUIRED)  #
find_package(sensor_msgs REQUIRED)  #
find_package(visualization_msgs REQUIRED)  #
find_package(geometry_msgs REQUIRED)  #
find_package(lifecycle_msgs REQUIRED)  #
find_package(tf2 REQUIRED)  #
find_package(tf2_ros REQUIRED)  #
find_package(tf2_geometry_msgs REQUIRED)  #
find_package(nav2_msgs REQUIRED)  #
find_package(can_plugins2 REQUIRED)  #
find_package(robomas_plugins REQUIRED)  #
find_package(nav2_amcl REQUIRED)  #
find_package(nhk24_utils REQUIRED)  #

#
include_directories(
  ../..
  include
)

# 
add_executable(r2_node
  src/r2_node.cpp
)
ament_target_dependencies(r2_node
  rclcpp
  std_msgs
  std_srvs
  lifecycle_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  sensor_msgs
  can_plugins2
  robomas_plugins
  nhk24_utils
)

# 
add_executable(filter_node
  src/filter_node.cpp
)
ament_target_dependencies(filter_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  can_plugins2
  nav2_amcl
)

# 
add_executable(collect_debug_node
  src/collect_debug_node.cpp
)
ament_target_dependencies(collect_debug_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  can_plugins2
  robomas_plugins
  nav2_amcl
)

#
add_executable(silo_position_publish_node
  src/silo_position_publish_node.cpp
)
ament_target_dependencies(silo_position_publish_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 
add_executable(hoge_node
  src/hoge.cpp
)
ament_target_dependencies(hoge_node
  rclcpp
  std_msgs
  std_srvs
  nav2_msgs
  lifecycle_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  can_plugins2
  sensor_msgs
)

# 
install(
  TARGETS
    r2_node  #
    filter_node  #
    collect_debug_node  #
    silo_position_publish_node  #
    hoge_node  #
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

ament_package()
