# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vision: 9 messages, 0 services")

set(MSG_I_FLAGS "-Ivision:/home/leokim/interface_ws/src/interface_ws/vision/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vision_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/white.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/white.msg" ""
)

get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/center.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/center.msg" ""
)

get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/black.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/black.msg" ""
)

get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/scan.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/scan.msg" ""
)

get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/colorbutton.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/colorbutton.msg" ""
)

get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/color.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/color.msg" ""
)

get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/camera.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/camera.msg" ""
)

get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/parametercheck.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/parametercheck.msg" ""
)

get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/parameterbutton.msg" NAME_WE)
add_custom_target(_vision_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vision" "/home/leokim/interface_ws/src/interface_ws/vision/msg/parameterbutton.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/colorbutton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/center.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/black.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/scan.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/white.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/color.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/camera.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/parametercheck.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)
_generate_msg_cpp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/parameterbutton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
)

### Generating Services

### Generating Module File
_generate_module_cpp(vision
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vision_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vision_generate_messages vision_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/white.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/center.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/black.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/scan.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/colorbutton.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/color.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/camera.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/parametercheck.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/parameterbutton.msg" NAME_WE)
add_dependencies(vision_generate_messages_cpp _vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_gencpp)
add_dependencies(vision_gencpp vision_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/colorbutton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/center.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/black.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/scan.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/white.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/color.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/camera.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/parametercheck.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)
_generate_msg_lisp(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/parameterbutton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
)

### Generating Services

### Generating Module File
_generate_module_lisp(vision
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vision_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vision_generate_messages vision_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/white.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/center.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/black.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/scan.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/colorbutton.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/color.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/camera.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/parametercheck.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/parameterbutton.msg" NAME_WE)
add_dependencies(vision_generate_messages_lisp _vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_genlisp)
add_dependencies(vision_genlisp vision_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/colorbutton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/center.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/black.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/scan.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/white.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/color.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/camera.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/parametercheck.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)
_generate_msg_py(vision
  "/home/leokim/interface_ws/src/interface_ws/vision/msg/parameterbutton.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
)

### Generating Services

### Generating Module File
_generate_module_py(vision
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vision_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vision_generate_messages vision_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/white.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/center.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/black.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/scan.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/colorbutton.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/color.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/camera.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/parametercheck.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/leokim/interface_ws/src/interface_ws/vision/msg/parameterbutton.msg" NAME_WE)
add_dependencies(vision_generate_messages_py _vision_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vision_genpy)
add_dependencies(vision_genpy vision_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vision_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vision
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(vision_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vision
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(vision_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vision
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(vision_generate_messages_py std_msgs_generate_messages_py)
