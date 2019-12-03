# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "cam: 0 messages, 1 services")

set(MSG_I_FLAGS "-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(cam_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv" NAME_WE)
add_custom_target(_cam_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "cam" "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(cam
  "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cam
)

### Generating Module File
_generate_module_cpp(cam
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cam
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(cam_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(cam_generate_messages cam_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(cam_generate_messages_cpp _cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cam_gencpp)
add_dependencies(cam_gencpp cam_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cam_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(cam
  "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cam
)

### Generating Module File
_generate_module_eus(cam
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cam
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(cam_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(cam_generate_messages cam_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(cam_generate_messages_eus _cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cam_geneus)
add_dependencies(cam_geneus cam_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cam_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(cam
  "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cam
)

### Generating Module File
_generate_module_lisp(cam
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cam
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(cam_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(cam_generate_messages cam_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(cam_generate_messages_lisp _cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cam_genlisp)
add_dependencies(cam_genlisp cam_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cam_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(cam
  "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cam
)

### Generating Module File
_generate_module_nodejs(cam
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cam
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(cam_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(cam_generate_messages cam_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(cam_generate_messages_nodejs _cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cam_gennodejs)
add_dependencies(cam_gennodejs cam_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cam_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(cam
  "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cam
)

### Generating Module File
_generate_module_py(cam
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cam
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(cam_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(cam_generate_messages cam_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa19/class/ee106a-abk/ros_workspaces/EE106A/workspaces/src/cam/srv/ImageSrv.srv" NAME_WE)
add_dependencies(cam_generate_messages_py _cam_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(cam_genpy)
add_dependencies(cam_genpy cam_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS cam_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/cam
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(cam_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/cam
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(cam_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/cam
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(cam_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cam)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/cam
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(cam_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cam)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cam\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/cam
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(cam_generate_messages_py sensor_msgs_generate_messages_py)
endif()
