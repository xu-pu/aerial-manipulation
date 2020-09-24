cmake_minimum_required(VERSION 2.8.3)

project(am_traj)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  quadrotor_msgs
  uav_utils
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    /home/sheep/am_ws/devel/include # for clion only
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES image_receiver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

#add_executable(${PROJECT_NAME}_node src/example0_node.cpp)

#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
#)
