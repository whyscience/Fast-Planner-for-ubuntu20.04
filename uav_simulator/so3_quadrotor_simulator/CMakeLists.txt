CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(so3_quadrotor_simulator)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include/ode include)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

set(dependencies rclcpp sensor_msgs quadrotor_msgs uav_utils Eigen3)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
ament_target_dependencies(quadrotor_dynamics ${dependencies})

add_executable(quadrotor_simulator_so3
        src/quadrotor_simulator_so3.cpp)

target_link_libraries(quadrotor_simulator_so3
        ${ARMADILLO_LIBRARIES}
        quadrotor_dynamics
        )

ament_target_dependencies(quadrotor_simulator_so3 ${dependencies})

