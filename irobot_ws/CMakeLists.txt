cmake_minimum_required(VERSION 3.10)

project(irobot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
# 动态库
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
# 静态库
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# pcl库依赖
find_package(PCL REQUIRED COMPONENTS common io visualization filters segmentation)

# 添加pcl头文件和库文件
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})

# opencv
find_package(OpenCV 4.5.5 REQUIRED COMPONENTS core highgui imgproc ximgproc)

# 添加eigen库
find_package(Eigen3 REQUIRED)

add_executable(main 
    src/main.cpp
)

target_include_directories(main
PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

add_library(core 
    src/baselib.cpp
    src/type/pose2d.cpp
    src/map/grid2d.cpp
    src/map/submap.cpp
    src/hough_scan_match.cpp
)

target_include_directories(core
PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(core
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)

target_link_libraries(main 
PRIVATE 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    core
)