# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "teleop_vision: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iteleop_vision:/home/charm/development/ros_ws/src/teleop-vision/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(teleop_vision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg" NAME_WE)
add_custom_target(_teleop_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teleop_vision" "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg" ""
)

get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv" NAME_WE)
add_custom_target(_teleop_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "teleop_vision" "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_vision
)

### Generating Services
_generate_srv_cpp(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_vision
)

### Generating Module File
_generate_module_cpp(teleop_vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(teleop_vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(teleop_vision_generate_messages teleop_vision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg" NAME_WE)
add_dependencies(teleop_vision_generate_messages_cpp _teleop_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv" NAME_WE)
add_dependencies(teleop_vision_generate_messages_cpp _teleop_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_vision_gencpp)
add_dependencies(teleop_vision_gencpp teleop_vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_vision_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_vision
)

### Generating Services
_generate_srv_eus(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_vision
)

### Generating Module File
_generate_module_eus(teleop_vision
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_vision
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(teleop_vision_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(teleop_vision_generate_messages teleop_vision_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg" NAME_WE)
add_dependencies(teleop_vision_generate_messages_eus _teleop_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv" NAME_WE)
add_dependencies(teleop_vision_generate_messages_eus _teleop_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_vision_geneus)
add_dependencies(teleop_vision_geneus teleop_vision_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_vision_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_vision
)

### Generating Services
_generate_srv_lisp(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_vision
)

### Generating Module File
_generate_module_lisp(teleop_vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(teleop_vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(teleop_vision_generate_messages teleop_vision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg" NAME_WE)
add_dependencies(teleop_vision_generate_messages_lisp _teleop_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv" NAME_WE)
add_dependencies(teleop_vision_generate_messages_lisp _teleop_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_vision_genlisp)
add_dependencies(teleop_vision_genlisp teleop_vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_vision_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_vision
)

### Generating Services
_generate_srv_nodejs(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_vision
)

### Generating Module File
_generate_module_nodejs(teleop_vision
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_vision
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(teleop_vision_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(teleop_vision_generate_messages teleop_vision_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg" NAME_WE)
add_dependencies(teleop_vision_generate_messages_nodejs _teleop_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv" NAME_WE)
add_dependencies(teleop_vision_generate_messages_nodejs _teleop_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_vision_gennodejs)
add_dependencies(teleop_vision_gennodejs teleop_vision_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_vision_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_vision
)

### Generating Services
_generate_srv_py(teleop_vision
  "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_vision
)

### Generating Module File
_generate_module_py(teleop_vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(teleop_vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(teleop_vision_generate_messages teleop_vision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/msg/TaskState.msg" NAME_WE)
add_dependencies(teleop_vision_generate_messages_py _teleop_vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/charm/development/ros_ws/src/teleop-vision/srv/CalculateStereoCamsTransfromFromTopics.srv" NAME_WE)
add_dependencies(teleop_vision_generate_messages_py _teleop_vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(teleop_vision_genpy)
add_dependencies(teleop_vision_genpy teleop_vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS teleop_vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/teleop_vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(teleop_vision_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/teleop_vision
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(teleop_vision_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/teleop_vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(teleop_vision_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/teleop_vision
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(teleop_vision_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/teleop_vision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(teleop_vision_generate_messages_py geometry_msgs_generate_messages_py)
endif()
