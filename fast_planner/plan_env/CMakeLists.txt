cmake_minimum_required(VERSION 3.5)
project(plan_env)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wno-vla -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED COMPONENTS common io )
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/sdf_map.cpp
        src/obj_predictor.cpp
        src/edt_environment.cpp
        src/raycast.cpp
        )

set(dependencies
        rclcpp
        visualization_msgs
        cv_bridge
        message_filters
        geometry_msgs
        pcl_ros
        Eigen3
        )

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

add_executable(obj_generator
        src/obj_generator.cpp
        )
target_link_libraries(obj_generator
        ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(TARGETS
        obj_generator
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
