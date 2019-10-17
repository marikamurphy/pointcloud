cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_write)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} /home/bwilab/pointcloud/CSerial)
link_directories(${PCL_LIBRARY_DIRS} /home/bwilab/pointcloud/CSerial)
add_definitions(${PCL_DEFINITIONS})

#Bring the headers, such as Student.h into the project
include_directories(include)
 
#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")
 
add_executable(pcd_write ${SOURCES})



#add_executable (pcd_write pcd_write.cpp)
target_link_libraries (pcd_write ${PCL_LIBRARIES} libcserial.so)
