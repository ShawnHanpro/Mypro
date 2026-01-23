# pcl库依赖
find_package(PCL REQUIRED COMPONENTS common io visualization filters)

# 增加opencv依赖
find_package(OpenCV REQUIRED)

# 增加g2o
find_package(g2o REQUIRED)
include_directories(
    ${G2O_INCLUDE_DIRS}
)
# g2o 2020+ 使用的c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 添加pcl头文件和库文件
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})

# 添加eigen库
find_package(Eigen3 REQUIRED)

add_executable(slam slam.cpp)
add_executable(main main.cpp)
add_executable(pcd2octomap pcd2octomap.cpp)

# octomap
target_include_directories(pcd2octomap PRIVATE
    /home/shan2/Mypro/octomap/octomap/include/octomap
)


add_library(slamBase slamBase.cpp)
target_link_libraries( slamBase
${OpenCV_LIBS}
${PCL_LIBRARIES}
)

target_link_libraries(pcd2octomap
    slamBase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    /home/shan2/Mypro/octomap/build/octomap/liboctomap.so
    /home/shan2/Mypro/octomap/build/octomap/liboctomath.so
)

target_link_libraries( slam 
    slamBase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core
    g2o_stuff
    g2o_solver_csparse
    g2o_csparse_extension
    g2o_types_slam3d
)

target_link_libraries( main 
    slamBase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core
    g2o_stuff
    g2o_solver_csparse
    g2o_csparse_extension
    g2o_types_slam3d
)