cmake_minimum_required(VERSION 2.8.3)

project(message_generation)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(ament_cmake rosidl_default_generators)

ament_package(
  CONFIG_EXTRAS "message_generation-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

