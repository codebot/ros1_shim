macro(_prepend_path ARG_PATH ARG_FILES ARG_OUTPUT_VAR)
  cmake_parse_arguments(ARG "UNIQUE" "" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "_prepend_path() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()
  # todo, check for proper path, slasheds, etc
  set(${ARG_OUTPUT_VAR} "")
  foreach(_file ${ARG_FILES})
    set(_value ${ARG_PATH}/${_file})
    list(FIND ${ARG_OUTPUT_VAR} ${_value} _index)
    if(NOT ARG_UNIQUE OR _index EQUAL -1)
      list(APPEND ${ARG_OUTPUT_VAR} ${_value})
    endif()
  endforeach()
endmacro()

macro(add_message_files)
  cmake_parse_arguments(ARG "NOINSTALL" "DIRECTORY;BASE_DIR" "FILES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "add_message_files() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DIRECTORY)
    set(ARG_DIRECTORY "msg")
  endif()

  set(MESSAGE_DIR "${ARG_DIRECTORY}")
  if(NOT IS_ABSOLUTE "${MESSAGE_DIR}")
    set(MESSAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${MESSAGE_DIR}")
  endif()
  # override message directory (used by add_action_files())
  if(ARG_BASE_DIR)
    set(MESSAGE_DIR ${ARG_BASE_DIR})
  endif()

  if(NOT IS_DIRECTORY ${MESSAGE_DIR})
    message(FATAL_ERROR "add_message_files() directory not found: ${MESSAGE_DIR}")
  endif()

  if(${PROJECT_NAME}_GENERATE_MESSAGES)
    message(FATAL_ERROR "generate_messages() must be called after add_message_files()")
  endif()

  # if FILES are not passed search message files in the given directory
  # note: ARGV is not variable, so it can not be passed to list(FIND) directly
  set(_argv ${ARGV})
  list(FIND _argv "FILES" _index)
  if(_index EQUAL -1)
    file(GLOB ARG_FILES RELATIVE "${MESSAGE_DIR}" "${MESSAGE_DIR}/*.msg")
    list(SORT ARG_FILES)
  endif()
  _prepend_path(${MESSAGE_DIR} "${ARG_FILES}" FILES_W_PATH)

  list(APPEND ${PROJECT_NAME}_MESSAGE_FILES ${FILES_W_PATH})
  foreach(file ${FILES_W_PATH})
    assert_file_exists(${file} "message file not found")
  endforeach()

#  # remember path to messages to resolve them as dependencies
#  list(FIND ${PROJECT_NAME}_MSG_INCLUDE_DIRS_DEVELSPACE ${MESSAGE_DIR} _index)
#  if(_index EQUAL -1)
#    list(APPEND ${PROJECT_NAME}_MSG_INCLUDE_DIRS_DEVELSPACE ${MESSAGE_DIR})
#  endif()
#
#  if(NOT ARG_NOINSTALL)
#    # ensure that destination variables are initialized
#    #catkin_destinations()
#
#    list(APPEND ${PROJECT_NAME}_MSG_INCLUDE_DIRS_INSTALLSPACE ${ARG_DIRECTORY})
#    install(FILES ${FILES_W_PATH}
#      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${ARG_DIRECTORY})
#
#    _prepend_path("${ARG_DIRECTORY}" "${ARG_FILES}" FILES_W_PATH)
#    list(APPEND ${PROJECT_NAME}_INSTALLED_MESSAGE_FILES ${FILES_W_PATH})
#  endif()
endmacro()

