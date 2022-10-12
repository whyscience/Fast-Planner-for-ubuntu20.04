CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(odom_visualization)
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(pose_utils REQUIRED)

set(dependencies
        rclcpp
        visualization_msgs
        nav_msgs
        sensor_msgs
        quadrotor_msgs
        tf2_ros
        tf2
        tf2_geometry_msgs
        geometry_msgs
        pose_utils
        )

###########
## Build ##
###########
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(include)

## Declare a cpp executable
add_executable(odom_visualization src/odom_visualization.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(odom_visualization
        ${ARMADILLO_LIBRARIES}
        )

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_package()