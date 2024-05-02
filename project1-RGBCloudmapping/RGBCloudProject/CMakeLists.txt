cmake_minimum_required(VERSION 3.10.0)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(RGBCloudProject)

find_package(PCL  REQUIRED)
find_package(Eigen3 QUIET)
find_package(OpenCV 3 REQUIRED)


include_directories(include 
                    lib/LidarProcess 
                    ${PCL_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
#SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_library(rgb_lidar_process_lib
        lib/LidarProcess/rgblidar_process.cpp
        )
target_include_directories(rgb_lidar_process_lib PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries (rgb_lidar_process_lib ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (rgb_cloud test/rgb_cloud.cpp)
target_link_libraries (rgb_cloud ${PCL_LIBRARIES} ${OpenCV_LIBS} rgb_lidar_process_lib)
