cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-parameter -Wno-unused-but-set-variable")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
)

set(dependencies
        rclcpp
        std_msgs
        geometry_msgs
        quadrotor_msgs
        cv_bridge
        plan_env
        path_searching
        bspline
        bspline_opt
        traj_utils
        pcl_ros
        Eigen3
        )

add_executable(fast_planner_node
        src/fast_planner_node.cpp
        src/kino_replan_fsm.cpp
        src/topo_replan_fsm.cpp
        src/planner_manager.cpp
        )
ament_target_dependencies(fast_planner_node ${dependencies})

add_executable(traj_server src/traj_server.cpp)
ament_target_dependencies(traj_server ${dependencies})

