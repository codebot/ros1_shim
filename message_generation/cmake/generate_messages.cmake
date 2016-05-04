find_package(rosidl_cmake REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
macro(generate_messages)
  cmake_parse_arguments(ARG "" "" "DEPENDENCIES;LANGS" ${ARGN})
  message("HELLO I AM GENERATING YOUR MESSAGES: ${${PROJECT_NAME}_MESSAGE_FILES}")

  if(ARG_DEPENDENCIES)
    rosidl_generate_interfaces(${PROJECT_NAME}_gencpp
      ${${PROJECT_NAME}_MESSAGE_FILES}
      DEPENDENCIES "${ARG_DEPENDENCIES}"
    )
  else()
    rosidl_generate_interfaces(${PROJECT_NAME}_gencpp
      ${${PROJECT_NAME}_MESSAGE_FILES}
    )
  endif()
  #message("     HERE ARE YOUR CATKIN_LIBRARIES: [ ${catkin_LIBRARIES} ]")
  # maybe use get_default_rmw_implementation()   in rmw_implementation_cmake
  get_default_rmw_implementation(rmw_impl)
  message("default rmw: ${rmw_impl}")
  get_rmw_typesupport(typesupport_impl "${rmw_impl}" LANGUAGE "cpp")
  message("typesupport impl: ${typesupport_impl}")
  set(_typesupport_target ${PROJECT_NAME}_gencpp__${typesupport_impl})
  list(APPEND catkin_LIBRARIES ${_typesupport_target})
  # TODO (codebot): break up rosidl_target_interfaces so that we don't 
  # duplicate its functionality here. This is extremely fragile since any
  # change in rosidl_target_interfaces will cause breakage here.
  get_target_property(_include_directories ${_typesupport_target} INTERFACE_INCLUDE_DIRECTORIES)
  # it would be better to append this to catkin_INCLUDE_DIRS but in many cases
  # users will have already called include_directories(${catkin_INCLUDE_DIRS})
  # before calling generate_messages()
  include_directories(${_include_directories})
  #list(APPEND catkin_INCLUDE
  #rosidl_target_interfaces(${PROJECT_NAME}_gencpp    ${typesupport_impl} 
  #get_rmw_typesupport(
  #message("     NOW I HAVE APPENDED SOME STUFF: [ ${catkin_LIBRARIES} ]")
  list(APPEND ${PROJECT_NAME}_LIBRARIES ${_typesupport_target})
endmacro()
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/msg/shim
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/srv/shim
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
)
