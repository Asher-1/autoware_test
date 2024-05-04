cmake_minimum_required(VERSION 2.8.3)
project(gnss_projection)

find_package(PCL  REQUIRED)
find_package(Eigen3 QUIET)
find_package(OpenCV 3 REQUIRED)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gnss
  gnss_localizer
  ndt_cpu
  nmea_msgs
  roscpp
  std_msgs
  tf
  pcl_conversions
  pcl_ros  
  tf2_eigen
  eigen_conversions
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
  src/gnss_projection_node.cpp
)

target_include_directories(gnss_projection PRIVATE nodes/gnss_projection ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gnss_projection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(gnss_projection ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Install executables and/or libraries
install(TARGETS
    gnss_projection
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

# Install config
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install map data
install(DIRECTORY map_data/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/map_data
)