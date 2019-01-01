# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "phd: 8 messages, 7 services")

set(MSG_I_FLAGS "-Iphd:/home/mike/catkin_ws/src/phd/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(phd_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv" "sensor_msgs/PointField:sensor_msgs/PointCloud2:std_msgs/Header"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv" "sensor_msgs/PointField:sensor_msgs/PointCloud2:std_msgs/Header"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/calc_service.srv" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/srv/calc_service.srv" "sensor_msgs/PointField:sensor_msgs/PointCloud2:std_msgs/Header"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/marker_val.msg" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/msg/marker_val.msg" ""
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg" ""
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg" "phd/trajectory_point:phd/trajectory_msg"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv" "sensor_msgs/PointField:sensor_msgs/PointCloud2:phd/trajectory_point:std_msgs/Header:phd/trajectory_msg"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv" "sensor_msgs/PointField:phd/doctor_msg:sensor_msgs/PointCloud2:std_msgs/Header"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg" ""
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg" ""
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/empty.srv" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/srv/empty.srv" ""
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv" "sensor_msgs/PointField:sensor_msgs/PointCloud2:std_msgs/Header"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg" "sensor_msgs/PointField:sensor_msgs/PointCloud2:std_msgs/Header"
)

get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg" NAME_WE)
add_custom_target(_phd_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "phd" "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg" "phd/trajectory_point"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(phd
  "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_msg_cpp(phd
  "/home/mike/catkin_ws/src/phd/msg/marker_val.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_msg_cpp(phd
  "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_msg_cpp(phd
  "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_msg_cpp(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg"
  "${MSG_I_FLAGS}"
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_msg_cpp(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_msg_cpp(phd
  "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_msg_cpp(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)

### Generating Services
_generate_srv_cpp(phd
  "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_srv_cpp(phd
  "/home/mike/catkin_ws/src/phd/srv/calc_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_srv_cpp(phd
  "/home/mike/catkin_ws/src/phd/srv/empty.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_srv_cpp(phd
  "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_srv_cpp(phd
  "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_srv_cpp(phd
  "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)
_generate_srv_cpp(phd
  "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
)

### Generating Module File
_generate_module_cpp(phd
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(phd_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(phd_generate_messages phd_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/calc_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/marker_val.msg" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/empty.srv" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_cpp _phd_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phd_gencpp)
add_dependencies(phd_gencpp phd_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phd_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(phd
  "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_msg_lisp(phd
  "/home/mike/catkin_ws/src/phd/msg/marker_val.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_msg_lisp(phd
  "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_msg_lisp(phd
  "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_msg_lisp(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg"
  "${MSG_I_FLAGS}"
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_msg_lisp(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_msg_lisp(phd
  "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_msg_lisp(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)

### Generating Services
_generate_srv_lisp(phd
  "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_srv_lisp(phd
  "/home/mike/catkin_ws/src/phd/srv/calc_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_srv_lisp(phd
  "/home/mike/catkin_ws/src/phd/srv/empty.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_srv_lisp(phd
  "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_srv_lisp(phd
  "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_srv_lisp(phd
  "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)
_generate_srv_lisp(phd
  "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
)

### Generating Module File
_generate_module_lisp(phd
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(phd_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(phd_generate_messages phd_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/calc_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/marker_val.msg" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/empty.srv" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_lisp _phd_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phd_genlisp)
add_dependencies(phd_genlisp phd_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phd_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(phd
  "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_msg_py(phd
  "/home/mike/catkin_ws/src/phd/msg/marker_val.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_msg_py(phd
  "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_msg_py(phd
  "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_msg_py(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg"
  "${MSG_I_FLAGS}"
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_msg_py(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_msg_py(phd
  "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_msg_py(phd
  "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  "${MSG_I_FLAGS}"
  "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)

### Generating Services
_generate_srv_py(phd
  "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_srv_py(phd
  "/home/mike/catkin_ws/src/phd/srv/calc_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_srv_py(phd
  "/home/mike/catkin_ws/src/phd/srv/empty.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_srv_py(phd
  "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_srv_py(phd
  "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_srv_py(phd
  "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)
_generate_srv_py(phd
  "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
)

### Generating Module File
_generate_module_py(phd
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(phd_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(phd_generate_messages phd_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/localize_cloud.srv" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/thickness_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/calc_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/marker_val.msg" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/arm_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/marker_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_array.msg" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/simple_trajectory_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/doctor_cloud.srv" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_point.msg" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/cube_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/empty.srv" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/srv/accuracy_service.srv" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/doctor_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mike/catkin_ws/src/phd/msg/trajectory_msg.msg" NAME_WE)
add_dependencies(phd_generate_messages_py _phd_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(phd_genpy)
add_dependencies(phd_genpy phd_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS phd_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/phd
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(phd_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(phd_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
add_dependencies(phd_generate_messages_cpp sensor_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/phd
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(phd_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(phd_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
add_dependencies(phd_generate_messages_lisp sensor_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/phd
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(phd_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(phd_generate_messages_py actionlib_msgs_generate_messages_py)
add_dependencies(phd_generate_messages_py sensor_msgs_generate_messages_py)
