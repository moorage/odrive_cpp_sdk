cmake_minimum_required(VERSION 2.8.3)
project(odrive_cpp_sdk)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
  INCLUDE_DIRS include
  LIBRARIES odrive_cpp_sdk
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(odrive_cpp_sdk
  src/odrive_cpp_sdk.cpp
)

add_dependencies(odrive_cpp_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
