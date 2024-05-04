cmake_minimum_required(VERSION 3.0.2)
project(rgb_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# set(OpenCV_DIR /usr/share/OpenCV/)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  message_generation
  pcl_conversions
  pcl_ros
  tf2_eigen
  eigen_conversions
  )

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate services in the 'srv' folder
add_service_files(
  FILES
  getCameraInfo.srv
  PointCloudShader.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgb_mapping
#  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(load_params_node src/load_params_node.cpp)
target_link_libraries(load_params_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(load_params_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(load_sensors_data_node src/load_sensors_data_node.cpp)
target_link_libraries(load_sensors_data_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(load_sensors_data_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(pointcloud_shader_server src/pointcloud_shader_server.cpp)
target_link_libraries(pointcloud_shader_server ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(pointcloud_shader_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(rgb_mapping src/rgb_mapping.cpp)
target_link_libraries(rgb_mapping ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(rgb_mapping
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

## Install executables and/or libraries
install(TARGETS
        load_params_node
        load_sensors_data_node
        pointcloud_shader_server
        rgb_mapping
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

# Install config
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

