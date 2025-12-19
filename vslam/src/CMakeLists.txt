# pcl库依赖
find_package(PCL REQUIRED COMPONENTS common io)

# 增加opencv依赖
find_package(OpenCV REQUIRED)

# 添加头文件和库文件
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARY_DIRS})

# add_executable(main main.cpp)
add_executable(generate_pointcloud generatePointCloud.cpp)
add_executable(main main.cpp)

target_link_libraries( generate_pointcloud 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

add_library(slamBase slamBase.cpp)
target_link_libraries( slamBase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)