CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(so3_control)
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        quadrotor_msgs
        tf
        nodelet
        cmake_utils
        )
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_library(SO3Control src/SO3Control.cpp)
add_library(so3_control_nodelet src/so3_control_nodelet.cpp)

target_link_libraries(so3_control_nodelet
        ${catkin_LIBRARIES}
        SO3Control
        )