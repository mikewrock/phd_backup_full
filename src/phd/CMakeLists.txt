cmake_minimum_required(VERSION 2.8.3)
project(phd)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
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
  message_filters
  actionlib_msgs
  laser_assembler
  pcl_conversions 
  pcl_ros 
  libpcan
  husky_description
  qt_build
dynamic_reconfigure
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
generate_dynamic_reconfigure_options(
  cfg/seg_config.cfg
  cfg/param_config.cfg
  cfg/localize_config.cfg
  #...
)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   cube_msg.msg
   arm_msg.msg
   marker_val.msg
   trajectory_point.msg
   trajectory_msg.msg
   trajectory_array.msg
   trajectory_section.msg
   
 )
## Generate services in the 'srv' folder
 add_service_files(
   FILES
	localize_cloud.srv
	thickness_service.srv
	trajectory_service.srv
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
CATKIN_DEPENDS sensor_msgs tf urdf diagnostic_updater schunk_libm5api libntcan libpcan cob_srvs brics_actuator diagnostic_msgs control_msgs  message_runtime roscpp dynamic_reconfigure
)    

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(common/include include /home/mike/pcl-master/filters/include/pcl/filters ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(common/include include ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake) 
link_directories(${PCL_LIBRARY_DIRS})


# Definitions
add_definitions(-D_USE_LINUX_API)
#add_definitions(-D_USE_CUSTOM_INTPTR)

qt4_wrap_cpp(MOC_FILES
  include/control_panel.hpp
  include/vel_control.hpp
  include/control_dashboard.h
  include/marker_selector.h
)


set(SOURCE_FILES
  src/control_panel.cpp   
  src/vel_control.cpp   
  src/control_dashboard.cpp   
  src/marker_selector.cpp   
  src/bcap_common.c
  src/dn_com.c
  src/dn_common.c
  src/dn_device.c
  src/dn_socket.c
  src/dn_tcp.c
  src/dn_thread.c
  src/dn_udp.c
  src/bcap_client.c
  ${MOC_FILES}
)
set(${PROJECT_NAME}_UIS
  src/control_panel.ui
)
QT4_ADD_RESOURCES(QT_RESOURCES include/ui_resource.qrc)
qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

add_executable(cube_node src/cube_node.cpp)
add_executable(parse_node src/parse_file.cpp)
add_executable(test_node src/test_node.cpp)
add_executable(denso_node src/denso_node.cpp)
add_executable(joint_fusion_node src/joint_state_fusion.cpp)
add_executable(cloud_localizer src/pointcloud_localization_service.cpp)
add_executable(thickness_server src/thickness_service.cpp)
add_executable(trajectory_server src/trajectory_service.cpp)
add_executable(simple_trajectory_server src/simple_trajectory_service.cpp)
add_executable(saveselection src/saveselection.cpp)

## Declare a cpp library
 add_library(${PROJECT_NAME} 
   ${SOURCE_FILES}
   ${${PROJECT_NAME}_UIS_H}
   ${QT_RESOURCES} ${MOC_FILES}
 )
# Client library
add_library(bcap_client 
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

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(bcap_client ${catkin_LIBRARIES})

target_link_libraries(cube_node  ${catkin_LIBRARIES})
target_link_libraries(parse_node  ${catkin_LIBRARIES})
target_link_libraries(test_node  ${catkin_LIBRARIES})
target_link_libraries(cloud_localizer  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(thickness_server  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(trajectory_server  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(simple_trajectory_server  ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(joint_fusion_node  ${catkin_LIBRARIES})
target_link_libraries(denso_node bcap_client ${catkin_LIBRARIES})
target_link_libraries(saveselection  ${catkin_LIBRARIES})


set_target_properties(cube_node PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_ESD")


add_dependencies(cloud_localizer  phd_EXPORTED_TARGETS phd_generate_messages_cpp phd_gencpp phd_gencfg)
add_dependencies(thickness_server  phd_EXPORTED_TARGETS phd_generate_messages_cpp phd_gencpp phd_gencfg)
add_dependencies(trajectory_server  phd_EXPORTED_TARGETS phd_generate_messages_cpp phd_gencpp)
add_dependencies(simple_trajectory_server  phd_EXPORTED_TARGETS phd_generate_messages_cpp phd_gencpp)
add_dependencies(cube_node  phd_EXPORTED_TARGETS phd_generate_messages_cpp)
add_dependencies(denso_node  phd_EXPORTED_TARGETS phd_generate_messages_cpp)
#add_dependencies(control_panel  phd_EXPORTED_TARGETS phd_generate_messages_cpp phd_gencpp)

#add_dependencies(cloud_localizer ${PROJECT_NAME}_gencfg)


install(TARGETS 
${PROJECT_NAME}
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
