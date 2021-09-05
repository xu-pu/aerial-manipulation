cmake_minimum_required(VERSION 2.8.3)
project(rnw_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        message_filters
        tf
        roscpp
        sensor_msgs
        nav_msgs
        std_msgs
        quadrotor_msgs
        poly_traj
        am_traj
        n3ctrl
        message_generation
        uav_utils
        rnw_msgs
        dynamic_reconfigure
        djiros
        rosbag
)

find_package(Ceres REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entrybelow
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )
generate_dynamic_reconfigure_options(
        cfg/MocapCalib.cfg
        cfg/RNW.cfg
        cfg/TuningConeState.cfg
        cfg/VelAcc.cfg
        cfg/ThrustTest.cfg
        cfg/Rl.cfg
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rnw_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  /home/sheep/am_ws/devel/include # for clion only
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rnw_ros.cpp
# )

add_library(${PROJECT_NAME}
        src/rnw_utils.cpp
        src/cone_state_estimator.cpp
        src/swarm_interface.cpp
        src/rnw_planner_v2.cpp
        src/drone_interface.cpp
        src/cone_interface.cpp
        src/caging_state.cpp
        src/rl_agent_interface.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rnw_ros_node.cpp)

set(RNW_NODES
        pub_euler_angles_node
        pub_cone_state_node
        pub_cone_debug_info_node
        pub_obj_trace_node
        pub_energy_node
        pub_swarm_rnw_debug_info_node
        visualize_traj_node
        visualize_cone_node
        visualize_pos_cmd_node
        visualize_disturbance_node
        gamepad_trigger_node
        gamepad_drone_cmd_node
        dbg_atti_loop_node
        mocap_calib_node
        odom2tf_node
        traj2cmd_node
        test_swarm_planner_node
        fake_drone_node
        swarm_rnw_controller_node
        check_planning_assistant_node
        individual_flight_test_node
        pub_caging_state_node
        rnw_caging_node
        rnw_cable_node
        dbg_rnw_cable_node
        ekf_odom_node
        pub_simulated_sensors
        thrust_test_node
        pub_rnw_body_frame_node
        visualize_rl_node
        test_vel_cmd_node
        caging_rl_node
)

FOREACH(NODE ${RNW_NODES})
    add_executable(${NODE} nodes/${NODE}.cpp)
    target_link_libraries(${NODE} ${PROJECT_NAME} ${catkin_LIBRARIES} ${CERES_LIBRARIES})
    #add_dependencies(${NODE} ${PROJECT_NAME}_generate_messages_cpp)
    add_dependencies(${NODE} ${PROJECT_NAME}_gencfg)
ENDFOREACH()

add_executable(extract_thrust scripts/extract_thrust.cpp)
target_link_libraries(extract_thrust ${catkin_LIBRARIES})
add_dependencies(extract_thrust ${PROJECT_NAME}_gencfg)

add_executable(extract_rnw scripts/extract_rnw.cpp)
target_link_libraries(extract_rnw ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(extract_rnw ${PROJECT_NAME}_gencfg)

add_executable(extract_old_rnw scripts/extract_old_rnw.cpp)
target_link_libraries(extract_old_rnw ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(extract_old_rnw ${PROJECT_NAME}_gencfg)

add_executable(extract_rl_rnw scripts/extract_rl_rnw.cpp)
target_link_libraries(extract_rl_rnw ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(extract_rl_rnw ${PROJECT_NAME}_gencfg)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rnw_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
