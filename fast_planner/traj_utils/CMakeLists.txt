cmake_minimum_required(VERSION 3.5)
project(traj_utils)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/planning_visualization.cpp
        )

set(dependencies
        rclcpp
        std_msgs
        visualization_msgs
        cv_bridge
        bspline
        bspline_opt
        path_searching
        poly_traj
        pcl_ros
        Eigen3
        )

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)
ament_package()