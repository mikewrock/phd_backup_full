cmake_minimum_required(VERSION 2.8.3)
project(wrock)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  urdf 
  diagnostic_updater
  schunk_libm5api
  libntcan
  libpcan
  cob_srvs
  brics_actuator
  diagnostic_msgs
  control_msgs
  message_generation
  actionlib_msgs
  laser_assembler
  pcl_conversions 
  pcl_ros 
  libpcan
husky_description
) 
find_package(PCL 1.7 REQUIRED)
## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

include_directories(include/)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  include/${PROJECT_NAME}
)
link_directories(${catkin_LIBRARY_DIRS})

if(CMAKE_COMPILER_IS_GNUCXX)
  #add_definitions(-std=gnu++0x)
else()
  #add_definitions(-std=c++0x)
endif()
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
   cube_pose.msg
   joint_pose.msg
   cube_kin.msg
   joint_kin.msg
   cube_msg.msg
   arm_msg.msg
 )
## Generate services in the 'srv' folder
 add_service_files(
   FILES
#   Service2.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
  sensor_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
    DEPENDS # TODO
    INCLUDE_DIRS 
      common/include
      include
    LIBRARIES # TODO
      bcap_client
      bcap_server
CATKIN_DEPENDS sensor_msgs tf urdf diagnostic_updater schunk_libm5api libntcan libpcan cob_srvs brics_actuator diagnostic_msgs control_msgs  message_runtime roscpp
)    

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(common/include include /home/mike/pcl-master/filters/include/pcl/filters ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
# add_library(wrock
#   src/${PROJECT_NAME}/wrock.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(wrock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(wrock_node src/wrock_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(wrock_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(wrock_node
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
# install(TARGETS wrock wrock_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wrock.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


# Definitions
add_definitions(-D_USE_LINUX_API)
#add_definitions(-D_USE_CUSTOM_INTPTR)


qt4_wrap_cpp(MOC_FILES include/drive_widget.h include/teleop_panel.h)

set(SOURCE_FILES
src/teleop_panel.cpp src/drive_widget.cpp
  ${MOC_FILES}
)


add_executable(cube_state_publisher src/cube_state_publisher.cpp)
add_executable(cube_move src/cube_move.cpp)
add_executable(cube_demo src/cube_demo.cpp)
#add_executable(cube_node src/cube_node.cpp)
add_executable(laser_demo src/laser_demo.cpp)
add_executable(pcl src/pcl.cpp)
add_executable(pcf src/pcf.cpp)
add_executable(pco src/pco.cpp)
add_executable(savepcd src/savepcd.cpp)
add_executable(savemarker src/savemarker.cpp)
add_executable(spatial src/spatial.cpp)
add_executable(fileplay src/fileplay.cpp)
add_executable(matching src/matching.cpp)
add_executable(matching2 src/matching2.cpp)
add_executable(global_trans src/global_trans.cpp)
add_executable(cloud_parser src/cloud_parser.cpp)
add_executable(registration src/registration.cpp)
add_executable(master src/master.cpp)
add_executable(marker_node src/marker_node.cpp)

add_executable(arm_pub src/arm_state_publisher.cpp)
add_executable(trajectory src/trajectory.cpp)
add_executable(move src/movedensoarm.cpp)
add_executable(pubmarker src/pubmarker.cpp)


# Client library
add_library(bcap_clienta 
  src/bcap_common.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_thread.c
  src/dn_udp.c
  src/bcap_client.c
)
## Set the default path for built libraries to the "lib" directory
set(CATKIN_PACKAGE_LIB_DESTINATION ${PROJECT_SOURCE_DIR}/lib)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})
## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(bcap_clienta ${catkin_LIBRARIES})

target_link_libraries(cube_state_publisher ${catkin_LIBRARIES})
target_link_libraries(cube_move  ${catkin_LIBRARIES})
#target_link_libraries(cube_node  ${catkin_LIBRARIES})
target_link_libraries(cube_demo  ${catkin_LIBRARIES})
target_link_libraries(laser_demo  ${catkin_LIBRARIES})
target_link_libraries(pcl  ${catkin_LIBRARIES})
target_link_libraries(pcf  ${catkin_LIBRARIES})
target_link_libraries(pco  ${catkin_LIBRARIES})
target_link_libraries(savepcd  ${catkin_LIBRARIES})
target_link_libraries(savemarker  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(fileplay  ${catkin_LIBRARIES})
target_link_libraries(spatial  ${catkin_LIBRARIES})
target_link_libraries(registration  ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
target_link_libraries(master  ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
target_link_libraries(marker_node  ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
target_link_libraries(pubmarker  ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)
target_link_libraries(matching  ${catkin_LIBRARIES})
target_link_libraries(matching2  ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
target_link_libraries(global_trans  ${catkin_LIBRARIES})
target_link_libraries(cloud_parser  ${catkin_LIBRARIES})
target_link_libraries(arm_pub  ${catkin_LIBRARIES})
target_link_libraries(move  ${catkin_LIBRARIES})
target_link_libraries(trajectory  ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

set_target_properties(cube_move PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_ESD")
#set_target_properties(cube_node PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_ESD")
set_target_properties(registration PROPERTIES COMPILE_FLAGS "-Wno-deprecated")
set_target_properties(trajectory PROPERTIES COMPILE_FLAGS "-Wno-deprecated")
set_target_properties(master PROPERTIES COMPILE_FLAGS "-Wno-deprecated")
set_target_properties(marker_node PROPERTIES COMPILE_FLAGS "-Wno-deprecated")
set_target_properties(pubmarker PROPERTIES COMPILE_FLAGS "-Wno-deprecated")


add_dependencies(cube_move  wrock_EXPORTED_TARGETS wrock_generate_messages_cpp)
#add_dependencies(cube_node  wrock_EXPORTED_TARGETS wrock_generate_messages_cpp)
add_dependencies(cube_demo  wrock wrock_generate_messages_cpp)
add_dependencies(laser_demo  wrock wrock_generate_messages_cpp)
add_dependencies(move  wrock wrock_generate_messages_cpp)


install(TARGETS 
${PROJECT_NAME}
master laser_demo savemarker marker_node trajectory pubmarker
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
