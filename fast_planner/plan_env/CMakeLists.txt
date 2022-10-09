cmake_minimum_required(VERSION 3.5)
project(plan_env)
set(CMAKE_CXX_STANDARD 14)

cmake_minimum_required(VERSION 3.5)
project(plan_env)

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
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS})

add_library(plan_env
        src/sdf_map.cpp
        src/obj_predictor.cpp
        src/edt_environment.cpp
        src/raycast.cpp
        )
ament_target_dependencies(plan_env rclcpp visualization_msgs cv_bridge message_filters geometry_msgs)
target_link_libraries(plan_env ${PCL_LIBRARIES})

add_executable(obj_generator
        src/obj_generator.cpp
        )
target_link_libraries(obj_generator
        plan_env
        ${PCL_LIBRARIES})

install(TARGETS
        plan_env
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        obj_generator
        DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(plan_env)
ament_export_dependencies(rclcpp visualization_msgs cv_bridge message_filters geometry_msgs)
ament_package()
