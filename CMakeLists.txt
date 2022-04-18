
cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(
  Field-Aided-RRT
  DESCRIPTION "Implementation of Field-Aided RRT for path planning in a partially observable environment"
  LANGUAGES CXX
  )
function(dump_cmake_variables)
  get_cmake_property(_variableNames VARIABLES)
  list (SORT _variableNames)
  foreach (_variableName ${_variableNames})
    if (ARGV0)
      unset(MATCHED)
      string(REGEX MATCH ${ARGV0} MATCHED ${_variableName})
      if (NOT MATCHED)
        continue()
      endif()
    endif()
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
endfunction()

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

  # Optionally set things like CMAKE_CXX_STANDARD, CMAKE_POSITION_INDEPENDENT_CODE here

  # Let's ensure -std=c++xx instead of -std=g++xx
  set(CMAKE_CXX_EXTENSIONS OFF)

  # Let's nicely support folders in IDEs
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

endif()

# Add TBB to path variables
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}:/usr/local/opt/tbb@2020_u3)
set(ENV{CPATH} $ENV{CPATH}:/usr/local/opt/tbb@2020_u3/include)
set(ENV{LIBRARY_PATH} $ENV{LIBRARY_PATH}:/usr/local/opt/tbb@2020_u3/lib)

# Find pacakge dependencies
# Adds gazebo::gazebo
find_package(gazebo REQUIRED)
message(STATUS "gazebo::gazebo version: ${gazebo_VERSION}")
#dump_cmake_variables()
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