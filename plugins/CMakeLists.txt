set (plugins
  world_edit
)

set (PLUGIN_OUTPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/compiled)
if (NOT EXISTS ${PLUGIN_OUTPUT_DIR})
  file(MAKE_DIRECTORY ${PLUGIN_OUTPUT_DIR})
  # execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${PLUGIN_OUTPUT_DIR})
endif ()

foreach (plugin_dir ${plugins})
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11.0")
      file(GLOB_RECURSE ${plugin_dir}_SRCS CONFIGURE_DEPENDS ${plugin_dir}/*.cpp ${plugin_dir}/*.cc)
  else()
      file(GLOB ${plugin_dir}_SRCS ${plugin_dir}/*.cpp ${plugin_dir}/*.cc)
  endif()
  # file(GLOB ${plugin_dir}_SRCS ${plugin_dir}/*.cpp ${plugin_dir}/*.cc)
  add_library(${plugin_dir} SHARED ${${plugin_dir}_SRCS})

# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})
# list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

  target_compile_options(${plugin_dir} PRIVATE ${GAZEBO_CXX_FLAGS})
  target_include_directories(${plugin_dir} PUBLIC ${GAZEBO_INCLUDE_DIRS})
  target_link_libraries(${plugin_dir} ${GAZEBO_LIBRARIES})

  # optionally copy built plugins back into source tree
  # add_custom_command(TARGET ${plugin_dir}
  #                 POST_BUILD
  #                 COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${plugin_dir}> ${PLUGIN_OUTPUT_DIR})
endforeach()