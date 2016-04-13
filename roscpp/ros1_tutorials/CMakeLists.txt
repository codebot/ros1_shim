cmake_minimum_required(VERSION 2.8.3)
project(ros1_chatter)

add_definitions(-std=c++11)
find_package(ros1_shim REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${ros1_shim_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

add_executable(talker talker.cpp)
target_link_libraries(talker ${ros1_shim_LIBRARIES} ${std_msgs_LIBRARIES})

add_executable(listener listener.cpp)
target_link_libraries(listener ${ros1_shim_LIBRARIES} ${std_msgs_LIBRARIES})
