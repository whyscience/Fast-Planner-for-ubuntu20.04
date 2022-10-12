CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
project(uav_utils)

## Turn on verbose output
set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_CXX_STANDARD 14)
## Add some compile flags
set(ADDITIONAL_CXX_FLAG "-Wall -O3 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED) # try to find manually installed eigen (Usually in /usr/local with provided FindEigen3.cmake)

include_directories(
        include
        ${EIGEN_INCLUDE_DIRS}
)

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
#endif()

#install(PROGRAMS scripts/
#        DESTINATION lib/${PROJECT_NAME}
#        )

install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)
ament_package()
