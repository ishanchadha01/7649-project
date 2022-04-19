cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  find_package(gazebo REQUIRED)
  if(NOT gazebo_FOUND)
    message(FATAL_ERROR "Gazebo is required to build this project")
  endif()
endif()

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11.0")
  file(GLOB_RECURSE listener_SRCS CONFIGURE_DEPENDS *.cpp *.cc)
else()
  file(GLOB listener_SRCS *.cpp *.cc)
endif()

add_executable(listener ${listener_SRCS})
target_link_directories(listener PUBLIC ${GAZEBO_LIBRARY_DIRS})
target_include_directories(listener PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(listener ${GAZEBO_LIBRARIES} pthread)

add_custom_command(TARGET listener
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:listener> ${PROJECT_SOURCE_DIR})