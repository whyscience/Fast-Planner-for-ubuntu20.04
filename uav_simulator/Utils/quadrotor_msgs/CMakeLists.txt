cmake_minimum_required(VERSION 3.5)
project(quadrotor_msgs)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
        msg/AuxCommand.msg
        msg/Corrections.msg
        msg/Gains.msg
        msg/OutputData.msg
        msg/PositionCommand.msg
        msg/PPROutputData.msg
        msg/Serial.msg
        msg/SO3Command.msg
        msg/StatusData.msg
        msg/TRPYCommand.msg
        msg/Odometry.msg
        msg/PolynomialTrajectory.msg
        msg/LQRTrajectory.msg
        msg/Bspline.msg
        )

rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES
        geometry_msgs
        nav_msgs
        )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
