cmake_minimum_required(VERSION 2.8.3)
project(genmsg)
install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


find_package(ament_cmake REQUIRED)

ament_export_dependencies(message_generation)

ament_package()
