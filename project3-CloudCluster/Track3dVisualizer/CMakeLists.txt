cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(Track3dVisualizer)

find_package(PCL 1.2 REQUIRED)
find_package(Eigen3)

include_directories(${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${EIGEN3_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable (run_test src/run_test.cpp src/VisualizerSet.cpp src/PointsCloudProcess.cpp)
target_link_libraries (run_test ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})




