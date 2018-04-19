cmake_minimum_required(VERSION 3.9)
project(mean_shift)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 11)

add_executable(mean_shift main.cpp cirlce.h circle.cpp)

target_link_libraries(mean_shift ${PCL_LIBRARIES})