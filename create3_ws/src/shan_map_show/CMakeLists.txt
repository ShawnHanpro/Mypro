cmake_minimum_required(VERSION 3.8)
project(shan_map_show)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 使用opencv4.5.5
set(OpenCV_DIR "/opt/opencv-4.5.5/lib/cmake/opencv4")
find_package(OpenCV 4.5.5 REQUIRED COMPONENTS core highgui imgproc ximgproc)
message("OpenCV version  is: ${OpenCV_VERSION}")

# 查找 MRPT 系统安装
find_package(MRPT REQUIRED maps serialization)

add_executable(shan_map_show_node src/shan_map_show_node.cpp)
target_include_directories(shan_map_show_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(shan_map_show_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  shan_map_show_node
  "rclcpp"
)

# 链接库
target_link_libraries(shan_map_show_node
    ${MRPT_LIBRARIES}
)

install(TARGETS shan_map_show_node
  DESTINATION lib/${PROJECT_NAME})

# 使用opencv4.5.5
target_link_libraries(shan_map_show_node ${OpenCV_LIBS})
set_target_properties(
  shan_map_show_node PROPERTIES
  INSTALL_RPATH "/opt/opencv-4.5.5/lib"
  INSTALL_RPATH_USE_LINK_PATH TRUE
  BUILD_WITH_INSTALL_RPATH TRUE  # 关键：确保在构建目录运行也能找到库
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
