CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(so3_control)
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(nodelet REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(SO3Control src/SO3Control.cpp)
add_library(so3_control_nodelet src/so3_control_nodelet.cpp)

set(dependencies
        rclcpp
        nav_msgs
        tf2_ros
        quadrotor_msgs
        Eigen3
        )

ament_target_dependencies(SO3Control ${dependencies})
ament_target_dependencies(so3_control_nodelet ${dependencies})

ament_export_include_directories(include)
ament_export_libraries(SO3Control)
ament_export_libraries(so3_control_nodelet)
ament_export_dependencies(${dependencies})

install(TARGETS SO3Control
        EXPORT SO3Control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(TARGETS so3_control_nodelet
        EXPORT so3_control_nodelet
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

ament_package()