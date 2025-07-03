cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(plane_octree)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable (plane_octree main.cpp)
target_link_libraries (plane_octree ${PCL_LIBRARIES})
