cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(openni_viewer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Bring the headers, such as Student.h into the project
include_directories(include)
 
#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "src/*.cpp")
 
add_executable(openni_viewer ${SOURCES})



#add_executable (openni_viewer openni_viewer_simple.cpp)
target_link_libraries (openni_viewer ${PCL_LIBRARIES})
