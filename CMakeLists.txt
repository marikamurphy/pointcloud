cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf
)

#include PCL directories
include_directories(${PCL_INCLUDE_DIRS} )
#include ROS directories (this is to find the AR tag
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3/
)

link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})

#Bring the headers, such as Student.h into the project
include_directories(include)
 
#However, the file(GLOB...) allows for wildcard additions:
file(GLOB WRITE "src/write/*.cpp")
add_executable(pcd_write ${WRITE})
#add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES} )

file(GLOB MERGE "src/merge/*.cpp")
add_executable(pcd_merge ${MERGE})
target_link_libraries (pcd_merge ${PCL_LIBRARIES} )

file(GLOB ALVAR "src/alvar/*.cpp")
add_executable(find_alvar ${ALVAR})
target_link_libraries(find_alvar
  ${catkin_LIBRARIES}
)


