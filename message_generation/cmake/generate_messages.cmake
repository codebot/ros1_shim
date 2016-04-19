macro(generate_messages)
  cmake_parse_arguments(ARG "" "" "DEPENDENCIES;LANGS" ${ARGN})

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
