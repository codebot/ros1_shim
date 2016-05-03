find_package(rosidl_cmake REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
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
endmacro()
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/msg/shim
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}/srv/shim
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
)
