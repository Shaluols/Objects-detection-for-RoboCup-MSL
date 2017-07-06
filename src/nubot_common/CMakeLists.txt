cmake_minimum_required(VERSION 2.8.3)
project(nubot_common)

set(CMAKE_BUILD_TYPE "Release")
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(catkin REQUIRED COMPONENTS
  message_filters
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
)


add_message_files(DIRECTORY msgs FILES Angle.msg
Point2d.msg
PPoint.msg
Point3d.msg
RobotInfo.msg
MotorInfo.msg
BallInfo.msg
BallInfo3d.msg
ObstaclesInfo.msg
ObstaclesInfo3d.msg
OminiVisionInfo.msg
object_info.msg

currentCmd.msg

VelCmd.msg
OdoInfo.msg
CoachInfo.msg
PassCommands.msg
WorldModelInfo.msg
StrategyInfo.msg
TargetInfo.msg
FrontBallInfo.msg
simulation_strategy.msg
)

add_service_files(DIRECTORY srv FILES BallHandle.srv Shoot.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

set(nubot_common_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/core/include 
)
include_directories(${nubot_common_includes})
catkin_package(
  INCLUDE_DIRS ${nubot_common_includes}
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/core/include
)
