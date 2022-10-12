CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(local_sensing_node)

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_STANDARD 14)

set(ENABLE_CUDA false)
# set(ENABLE_CUDA true)

if (ENABLE_CUDA)
    find_package(CUDA REQUIRED)
    SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
    set(CUDA_NVCC_FLAGS
            #       -gencode arch=compute_20,code=sm_20;
            #       -gencode arch=compute_20,code=sm_21;
            #       -gencode arch=compute_30,code=sm_30;
            #       -gencode arch=compute_35,code=sm_35;
            #       -gencode arch=compute_50,code=sm_50;
            #       -gencode arch=compute_52,code=sm_52;
            #       -gencode arch=compute_60,code=sm_60;
            -gencode arch=compute_61,code=sm_61;
            )

    SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

    find_package(OpenCV REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(Boost REQUIRED COMPONENTS system filesystem)

    find_package(
            rclcpp roslib cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs dynamic_reconfigure)
    generate_dynamic_reconfigure_options(
            cfg/local_sensing_node.cfg
    )

    include_directories(
            include
            ${OpenCV_INCLUDE_DIRS}
            ${Eigen_INCLUDE_DIRS}
            ${Boost_INCLUDE_DIRS}
    )

    CUDA_ADD_LIBRARY(depth_render_cuda
            src/depth_render.cu
            )

    add_executable(
            pcl_render_node
            src/pcl_render_node.cpp
    )
    target_link_libraries(pcl_render_node
            depth_render_cuda
            ${OpenCV_LIBS}
            ${Boost_LIBRARIES}
            )
else (ENABLE_CUDA)
    #todo temp
    find_package(Eigen3 REQUIRED)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(pcl_ros REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(quadrotor_msgs REQUIRED)

    include_directories(include ${Eigen_INCLUDE_DIRS})

    add_executable(
            pcl_render_node
            src/pointcloud_render_node.cpp
    )

    set(dependencies
            rclcpp
            pcl_ros
            std_msgs
            geometry_msgs
            nav_msgs
            quadrotor_msgs
            Eigen3
            )

    ament_target_dependencies(pcl_render_node ${dependencies})
    ament_package()
endif (ENABLE_CUDA)
