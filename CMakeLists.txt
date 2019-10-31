cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} )
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


