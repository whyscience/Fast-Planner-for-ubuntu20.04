CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(pose_utils)
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
        include
        ${ARMADILLO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/pose_utils.cpp)

target_link_libraries(${PROJECT_NAME} ${ARMADILLO_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} rclcpp)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

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