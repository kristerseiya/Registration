cmake_minimum_required(VERSION 3.0)
project(visualizer)
set(CMAKE_CXX_STANDARD 11)

find_package(open3d REQUIRED)
message(STATUS "open3d include: ${Open3D_INCLUDE_DIRS}")
message(STATUS "open3d library: ${Open3D_LIBRARIES}")
include_directories(${Open3D_INCLUDE_DIRS})

add_executable(visualize visualizer.cpp)
target_link_libraries(visualize ${Open3D_LIBRARIES})
