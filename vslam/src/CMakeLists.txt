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

add_library(slamBase slamBase.cpp)
target_link_libraries( slamBase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
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
    # ${G2O_CORE_LIBRARY}
    # ${G2O_STUFF_LIBRARY}
    # ${G2O_TYPES_SBA}
    # ${G2O_TYPES_SLAM3D}
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
    # ${G2O_CORE_LIBRARY}
    # ${G2O_STUFF_LIBRARY}
    # ${G2O_TYPES_SBA}
    # ${G2O_TYPES_SLAM3D}
)