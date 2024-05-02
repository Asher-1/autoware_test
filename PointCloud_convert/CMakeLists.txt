cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(PointCloud_convert)

find_package(PCL  REQUIRED)
find_package(OpenMP)
find_package(Eigen3 QUIET)
find_package(OpenCV 3 REQUIRED)


include_directories(include ${PCL_INCLUDE_DIRS} {OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (TypeConvert TypeConvert.cpp)
target_link_libraries (TypeConvert ${PCL_LIBRARIES})
