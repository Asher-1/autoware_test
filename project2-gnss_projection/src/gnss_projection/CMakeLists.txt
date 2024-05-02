cmake_minimum_required(VERSION 2.8.3)
project(gnss_projection)

find_package(PCL  REQUIRED)
find_package(Eigen3 QUIET)
find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gnss
  ndt_cpu
  nmea_msgs
  roscpp
  std_msgs
  tf
)

catkin_package()

SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include 
                    ${PCL_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(gnss_projection
  nodes/gnss_projection/gnss_projection_core.cpp
  nodes/gnss_projection/gnss_projection_node.cpp
)

target_include_directories(gnss_projection PRIVATE nodes/gnss_projection ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gnss_projection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(gnss_projection ${catkin_EXPORTED_TARGETS})
