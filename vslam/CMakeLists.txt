cmake_minimum_required(VERSION 3.8) # 设定版本
project(vslam) # 工程名称
# set(CMAKE_CXX_COMPILER "g++") # 设定编译器

# 设定可执行的二进制文件目录
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 设定存放编译出来的库文件目录
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 并把该目录设为链接目录
link_directories(${PROJECT_BINARY_DIR}/lib)

# 设定头文件目录
include_directories(${PROJECT_SOURCE_DIR}/include)

# 增加子文件夹 也就是进入源码文件夹继续构建
add_subdirectory(src)