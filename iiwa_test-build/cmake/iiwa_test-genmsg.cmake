# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "iiwa_test: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iiiwa_test:/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(iiwa_test_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg" NAME_WE)
add_custom_target(_iiwa_test_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "iiwa_test" "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg" "std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray:std_msgs/Header:std_msgs/MultiArrayLayout"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(iiwa_test
  "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iiwa_test
)

### Generating Services

### Generating Module File
_generate_module_cpp(iiwa_test
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iiwa_test
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(iiwa_test_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(iiwa_test_generate_messages iiwa_test_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg" NAME_WE)
add_dependencies(iiwa_test_generate_messages_cpp _iiwa_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iiwa_test_gencpp)
add_dependencies(iiwa_test_gencpp iiwa_test_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iiwa_test_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(iiwa_test
  "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iiwa_test
)

### Generating Services

### Generating Module File
_generate_module_eus(iiwa_test
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iiwa_test
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(iiwa_test_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(iiwa_test_generate_messages iiwa_test_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg" NAME_WE)
add_dependencies(iiwa_test_generate_messages_eus _iiwa_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iiwa_test_geneus)
add_dependencies(iiwa_test_geneus iiwa_test_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iiwa_test_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(iiwa_test
  "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iiwa_test
)

### Generating Services

### Generating Module File
_generate_module_lisp(iiwa_test
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iiwa_test
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(iiwa_test_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(iiwa_test_generate_messages iiwa_test_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg" NAME_WE)
add_dependencies(iiwa_test_generate_messages_lisp _iiwa_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iiwa_test_genlisp)
add_dependencies(iiwa_test_genlisp iiwa_test_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iiwa_test_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(iiwa_test
  "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iiwa_test
)

### Generating Services

### Generating Module File
_generate_module_nodejs(iiwa_test
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iiwa_test
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(iiwa_test_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(iiwa_test_generate_messages iiwa_test_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg" NAME_WE)
add_dependencies(iiwa_test_generate_messages_nodejs _iiwa_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iiwa_test_gennodejs)
add_dependencies(iiwa_test_gennodejs iiwa_test_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iiwa_test_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(iiwa_test
  "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iiwa_test
)

### Generating Services

### Generating Module File
_generate_module_py(iiwa_test
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iiwa_test
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(iiwa_test_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(iiwa_test_generate_messages iiwa_test_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/charlie/Documents/workspace/ros_ws/src/stentgraft_planning/iiwa_test/msg/iiwaState.msg" NAME_WE)
add_dependencies(iiwa_test_generate_messages_py _iiwa_test_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(iiwa_test_genpy)
add_dependencies(iiwa_test_genpy iiwa_test_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS iiwa_test_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iiwa_test)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/iiwa_test
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(iiwa_test_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iiwa_test)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/iiwa_test
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(iiwa_test_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iiwa_test)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/iiwa_test
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(iiwa_test_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iiwa_test)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/iiwa_test
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(iiwa_test_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iiwa_test)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iiwa_test\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/iiwa_test
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(iiwa_test_generate_messages_py std_msgs_generate_messages_py)
endif()
