cmake_minimum_required(VERSION 3.5)
project(bspline_opt)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plan_env REQUIRED)

include_directories(
        include
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(bspline_opt
        src/bspline_optimizer.cpp
        )
ament_target_dependencies(bspline_opt rclcpp nlopt plan_env)

ament_export_include_directories(include)
ament_export_libraries(bspline_opt)
ament_export_dependencies(rclcpp nlopt plan_env)

install(TARGETS
        bspline_opt
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_package()



