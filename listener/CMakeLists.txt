cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(listener listener.cc)
target_link_libraries(listener ${GAZEBO_LIBRARIES} pthread)