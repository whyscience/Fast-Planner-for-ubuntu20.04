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

add_library(${PROJECT_NAME}
        src/bspline_optimizer.cpp
        )
target_link_libraries(${PROJECT_NAME} nlopt)
ament_target_dependencies(${PROJECT_NAME} rclcpp plan_env Eigen3)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp plan_env Eigen3)

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



