cmake_minimum_required(VERSION 2.8.3)
project(carlos_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp rospy std_msgs tf moveit_msgs mission_ctrl_msgs ftm_msgs message_generation eigen moveit_ros_planning_interface amn_common amn_welding_iface genmsg actionlib_msgs brics_actuator actionlib REQUIRED)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   StudsPoses.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   JointMove.srv
   PoseMove.srv
   RPYMove.srv
   FTMove.srv
   JointConstraints.srv
   CurrentPose.srv
   CurrentJoints.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  DoTest.action
#   Action2.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs  # Or other packages containing msgs
   brics_actuator
   geometry_msgs
   moveit_msgs
   std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lib/EposCmd
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(carlos_controller
#   src/${PROJECT_NAME}/carlos_controller.cpp
# )

## Declare a cpp executable
# add_executable(carlos_controller_node src/carlos_controller_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(carlos_controller_node carlos_controller_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(carlos_controller_node
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

## Mark executables and/or libraries for installation
# install(TARGETS carlos_controller carlos_controller_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_carlos_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

include_directories(include ${catkin_INCLUDE_DIRS} ${amn_common_INCLUDE_DIRS} ${ftm_msgs_INCLUDE_DIRS})

link_directories(./lib)

include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

add_executable(arm_interface src/arm_interface.cpp)
target_link_libraries(arm_interface ${catkin_LIBRARIES})

add_executable(server_test src/server_test.cpp)
target_link_libraries(server_test ${catkin_LIBRARIES})

add_executable(crl_velocity src/crl_velocity.cpp)
target_link_libraries(crl_velocity ${catkin_LIBRARIES})

add_executable(client1_test src/client1_test.cpp)
target_link_libraries(client1_test ${catkin_LIBRARIES})

add_executable(client2_test src/client2_test.cpp)
target_link_libraries(client2_test ${catkin_LIBRARIES})

add_executable(test_distribution src/test_distribution.cpp)
target_link_libraries(test_distribution ${catkin_LIBRARIES})

add_executable(test_projection src/test_projection.cpp)
target_link_libraries(test_projection ${catkin_LIBRARIES})

add_executable(test_welding src/test_welding.cpp)
target_link_libraries(test_welding ${catkin_LIBRARIES})

add_executable(crl_moveit src/crl_moveit.cpp)
target_link_libraries(crl_moveit ${catkin_LIBRARIES})

add_executable(crl_controller src/crl_controller.cpp src/crl_contact.cpp)
target_link_libraries(crl_controller ${catkin_LIBRARIES} amn_mnl_lib)

add_executable(studs_feeder src/studs_feeder.cpp)
target_link_libraries(studs_feeder ${catkin_LIBRARIES} EposCmd)

