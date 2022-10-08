cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  #armadillo
  roscpp
  nav_msgs
  geometry_msgs
  message_generation
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )


add_message_files(
  FILES
  AuxCommand.msg
  Corrections.msg
  Gains.msg
  OutputData.msg
  PositionCommand.msg
  PPROutputData.msg
  Serial.msg
  SO3Command.msg
  StatusData.msg
  TRPYCommand.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES irobot_msgs
  #CATKIN_DEPENDS geometry_msgs nav_msgs
  #DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

#add_dependencies(decode_msgs quadrotor_msgs_generate_messages_cpp)
#add_dependencies(encode_msgs quadrotor_msgs_generate_messages_cpp)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)
