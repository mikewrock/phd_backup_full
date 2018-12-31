# Install script for directory: /home/mike/catkin_ws/src/phd

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mike/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/phd" TYPE FILE FILES "/home/mike/catkin_ws/devel/include/phd/param_configConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/phd" TYPE FILE FILES "/home/mike/catkin_ws/devel/include/phd/localize_configConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/phd" TYPE FILE FILES "/home/mike/catkin_ws/devel/include/phd/trajectory_configConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/phd" TYPE FILE FILES "/home/mike/catkin_ws/devel/include/phd/seg_configConfig.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/phd" TYPE FILE FILES "/home/mike/catkin_ws/devel/lib/python2.7/dist-packages/phd/__init__.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/mike/catkin_ws/devel/lib/python2.7/dist-packages/phd/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages/phd" TYPE DIRECTORY FILES "/home/mike/catkin_ws/devel/lib/python2.7/dist-packages/phd/cfg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phd/msg" TYPE FILE FILES
    "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg"
    "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg"
    "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg"
    "/home/mike/catkin_ws/src/phd/msg/marker_val.msg"
    "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg"
    "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
    "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg"
    "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phd/srv" TYPE FILE FILES
    "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv"
    "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv"
    "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv"
    "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv"
    "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv"
    "/home/mike/catkin_ws/src/phd/srv/empty.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phd/cmake" TYPE FILE FILES "/home/mike/catkin_ws/build/phd/catkin_generated/installspace/phd-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/mike/catkin_ws/devel/include/phd")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/mike/catkin_ws/devel/share/common-lisp/ros/phd")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/mike/catkin_ws/devel/lib/python2.7/dist-packages/phd")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/mike/catkin_ws/devel/lib/python2.7/dist-packages/phd")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mike/catkin_ws/build/phd/catkin_generated/installspace/phd.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phd/cmake" TYPE FILE FILES "/home/mike/catkin_ws/build/phd/catkin_generated/installspace/phd-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phd/cmake" TYPE FILE FILES
    "/home/mike/catkin_ws/build/phd/catkin_generated/installspace/phdConfig.cmake"
    "/home/mike/catkin_ws/build/phd/catkin_generated/installspace/phdConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phd" TYPE FILE FILES "/home/mike/catkin_ws/src/phd/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}/home/mike/catkin_ws/src/phd/lib/libphd.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mike/catkin_ws/src/phd/lib/libphd.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/mike/catkin_ws/src/phd/lib/libphd.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/mike/catkin_ws/src/phd/lib/libphd.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/mike/catkin_ws/src/phd/lib" TYPE SHARED_LIBRARY FILES "/home/mike/catkin_ws/devel/lib/libphd.so")
  if(EXISTS "$ENV{DESTDIR}/home/mike/catkin_ws/src/phd/lib/libphd.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/mike/catkin_ws/src/phd/lib/libphd.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/home/mike/catkin_ws/src/phd/lib/libphd.so"
         OLD_RPATH "/opt/ros/indigo/lib:/home/mike/catkin_ws/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/mike/catkin_ws/src/phd/lib/libphd.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phd" TYPE FILE FILES "/home/mike/catkin_ws/src/phd/plugin_description.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/phd" TYPE DIRECTORY FILES
    "/home/mike/catkin_ws/src/phd/launch"
    "/home/mike/catkin_ws/src/phd/meshes"
    "/home/mike/catkin_ws/src/phd/urdf"
    )
endif()

