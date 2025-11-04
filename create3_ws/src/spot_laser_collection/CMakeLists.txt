cmake_minimum_required(VERSION 3.8)
project(spot_laser_collection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(spot_laser_collection_node src/spot_laser_collection_node.cpp)
target_include_directories(spot_laser_collection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(spot_laser_collection_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  spot_laser_collection_node
  "rclcpp"
  "nav_msgs"
  "sensor_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "irobot_create_msgs"
  "tf2"
)

install(TARGETS spot_laser_collection_node
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
  "msg/WheelVels.msg"
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
