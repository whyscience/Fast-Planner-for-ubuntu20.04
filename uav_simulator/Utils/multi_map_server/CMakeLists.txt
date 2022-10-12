CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(multi_map_server)
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pose_utils REQUIRED)
find_package(quadrotor_msgs REQUIRED)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
include_directories(include)

set(msg_files
        msg/MultiOccupancyGrid.msg
        msg/MultiSparseMap3D.msg
        msg/SparseMap3D.msg
        msg/VerticalOccupancyGridList.msg
        )

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs
        )

add_executable(multi_map_visualization src/multi_map_visualization.cc)
ament_target_dependencies(multi_map_visualization rclcpp geometry_msgs nav_msgs)
rosidl_target_interfaces(multi_map_visualization ${PROJECT_NAME} "rosidl_typesupport_cpp")
# this is rolling version
#rosidl_get_typesupport_target(cpp_typesupport_target multi_map_visualization "rosidl_typesupport_cpp")
target_link_libraries(multi_map_visualization ${ARMADILLO_LIBRARIES})

ament_export_dependencies(rosidl_default_runtime)
ament_package()