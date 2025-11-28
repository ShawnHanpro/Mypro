cmake_minimum_required(VERSION 3.8)
project(edge_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(nav_msgs REQUIRED)

# msg
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(edge_following_node src/edge_following_node.cpp)
target_include_directories(edge_following_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(edge_following_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  edge_following_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_sensor_msgs"
  "irobot_create_msgs"
  "laser_geometry"
  "nav_msgs"
)
install(TARGETS edge_following_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(dwa src/DWA.cpp)
target_include_directories(dwa PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dwa PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dwa
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_sensor_msgs"
  "irobot_create_msgs"
  "laser_geometry"
  "nav_msgs"
)
install(TARGETS dwa
  DESTINATION lib/${PROJECT_NAME})

# launch
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# yaml
install(
  DIRECTORY yaml
  DESTINATION share/${PROJECT_NAME}/
)

# 为接口文件生成源码
rosidl_generate_interfaces( ${PROJECT_NAME}
  "msg/HazardDetection.msg"
  "msg/HazardDetectionVector.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
