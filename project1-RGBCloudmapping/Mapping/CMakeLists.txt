cmake_minimum_required(VERSION 3.10.0)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(RGBCloudProject)

find_package(PCL  REQUIRED)
find_package(Eigen3 QUIET)
find_package(OpenCV 3 REQUIRED)


include_directories(include 
                    lib/Mapping
                    lib/ndt_cpu/include/ndt_cpu 
                    lib/ndt_cpu/include
                    ${PCL_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
#SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(srcs
  lib/ndt_cpu/src/NormalDistributionsTransform.cpp
  lib/ndt_cpu/src/Registration.cpp
  lib/ndt_cpu/src/VoxelGrid.cpp
  lib/ndt_cpu/src/Octree.cpp
)

set(incs
  lib/ndt_cpu/include/ndt_cpu/debug.h
  lib/ndt_cpu/include/ndt_cpu/NormalDistributionsTransform.h
  lib/ndt_cpu/include/ndt_cpu/Registration.h
  lib/ndt_cpu/include/ndt_cpu/SymmetricEigenSolver.h
  lib/ndt_cpu/include/ndt_cpu/VoxelGrid.h
  lib/ndt_cpu/include/ndt_cpu/Octree.h
)

add_library(ndt_cpu ${incs} ${srcs})

target_link_libraries(ndt_cpu
  ${PCL_LIBRARIES}
)


add_library(ndt_mapping_lib
            lib/Mapping/ndt_mapping.cpp
            )
target_include_directories(ndt_mapping_lib PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries (ndt_mapping_lib ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (mapping_test test/mapping_test.cpp)
target_link_libraries (mapping_test ${PCL_LIBRARIES} ${OpenCV_LIBS} ndt_mapping_lib ndt_cpu)
