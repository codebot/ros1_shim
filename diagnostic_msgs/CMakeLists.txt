cmake_minimum_required(VERSION 2.8.3)

project(diagnostic_msgs)

find_package(ament_cmake REQUIRED)

include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(diagnostics_msgs)

ament_package()
