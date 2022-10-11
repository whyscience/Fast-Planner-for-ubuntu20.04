CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
)

set(dependencies
        rclcpp
        std_msgs
        geometry_msgs
        pcl_conversions
        pcl_ros
        Eigen3
        )

add_executable(random_forest src/random_forest_sensing.cpp)
ament_target_dependencies(random_forest ${dependencies})
ament_package()