
cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(
  Field-Aided-RRT
  DESCRIPTION "Implementation of Field-Aided RRT for path planning in a partially observable environment"
  LANGUAGES CXX
  )


# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

  # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here

  # Let's ensure -std=c++xx instead of -std=g++xx
  set(CMAKE_CXX_EXTENSIONS OFF)

  # Let's nicely support folders in IDEs
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

endif()

# Find pacakge dependencies
# Adds gazebo::gazebo
find_package(gazebo REQUIRED)
message(STATUS "gazebo::gazebo version: ${gazebo_VERSION}")

# Compile all gazebo plugins
add_subdirectory(plugins)

# The executable code is here
# add_subdirectory(apps)


# Testing only available if this is the main app
# Emergency override MODERN_CMAKE_BUILD_TESTING provided as well
if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MODERN_CMAKE_BUILD_TESTING)
   AND BUILD_TESTING)
  add_subdirectory(tests)
endif()