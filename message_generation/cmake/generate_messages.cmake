macro(generate_messages)
  cmake_parse_arguments(ARG "" "" "DEPENDENCIES;LANGS" ${ARGN})

  if(ARG_DEPENDENCIES)
    set(_dep_arg "DEPENDENCIES ${ARG_DEPENDENCIES}")
  endif()
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${${PROJECT_NAME}_MESSAGE_FILES}
    # TODO
    #${_dep_arg}
  )
endmacro()
