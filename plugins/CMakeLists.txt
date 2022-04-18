set (plugins
  setup_arena
)

# set (PLUGIN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/compiled)
# if (NOT EXISTS ${PLUGIN_OUTPUT_DIR})
#   file(MAKE_DIRECTORY ${PLUGIN_OUTPUT_DIR})
#   # execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PLUGIN_OUTPUT_DIR})
# endif ()

foreach (plugin_name ${plugins})
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11.0")
      file(GLOB_RECURSE ${plugin_name}_SRCS CONFIGURE_DEPENDS ${plugin_name}/*.cpp ${plugin_name}/*.cc)
  else()
      file(GLOB ${plugin_name}_SRCS ${plugin_name}/*.cpp ${plugin_name}/*.cc)
  endif()
  # file(GLOB ${plugin_name}_SRCS ${plugin_name}/*.cpp ${plugin_name}/*.cc)
  add_library(${plugin_name} SHARED ${${plugin_name}_SRCS})

# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")



  #target_compile_options(${plugin_name} PRIVATE ${GAZEBO_CXX_FLAGS})
  target_include_directories(${plugin_name} PUBLIC ${GAZEBO_INCLUDE_DIRS})
  target_link_libraries(${plugin_name} ${GAZEBO_LIBRARIES})
  # target_link_options(${plugin_name} PUBLIC ${GAZEBO_LDFLAGS})
  # target_link_options(${plugin_name} PUBLIC -v)

  target_link_directories(${plugin_name} PUBLIC ${GAZEBO_LIBRARY_DIRS})

  # optionally copy built plugins back into source tree
  # add_custom_command(TARGET ${plugin_name}
  #                 POST_BUILD
  #                 COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${plugin_name}> ${PLUGIN_OUTPUT_DIR})
endforeach()