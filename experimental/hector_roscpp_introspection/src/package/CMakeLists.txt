if(NOT DEFINED package)
  message(SEND_FATAL "You have to define variable package to determine the package for which introspection library is going to be built.")
  return()
endif(NOT DEFINED package)

# call rospack to find the package path
rosbuild_find_ros_package(${package})
if(NOT EXISTS "${${package}_PACKAGE_PATH}")
  message(FATAL_ERROR "Package ${package} not found in ROS_PACKAGE_PATH, cannot build introspection library")
  return()
endif()
#message(STATUS "Found package ${package} in ${${package}_PACKAGE_PATH}")

# enumerate messages in ${package}
file(GLOB MSG_FILES RELATIVE "${${package}_PACKAGE_PATH}/msg" "${${package}_PACKAGE_PATH}/msg/*.msg")
if("${MSG_FILES}" STREQUAL "")
  if(NOT QUIET)
    message(SEND_ERROR "Package ${package} contains no message definitions, cannot build introspection library")
  endif(NOT QUIET)
  return()
endif()

# Get the include dirs
rosbuild_invoke_rospack(${package} ${package} INCLUDE_DIRS cflags-only-I)
#message("${package} include dirs: ${${package}_INCLUDE_DIRS}")
include_directories(${${package}_INCLUDE_DIRS})

# Get the other cflags
#rosbuild_invoke_rospack(${package} ${package} temp cflags-only-other)
#_rosbuild_list_to_string(${package}_CFLAGS_OTHER "${${package}_temp}")
#message("${package} other cflags: ${${package}_CFLAGS_OTHER}")

# Get the lib dirs
#rosbuild_invoke_rospack(${package} ${package} LIBRARY_DIRS libs-only-L)
#message("${package} library dirs: ${${package}_LIBRARY_DIRS}")
#link_directories(${${package}_LIBRARY_DIRS})

# Get the libs
#rosbuild_invoke_rospack(${package} ${package} LIBRARIES libs-only-l)

#
# The following code removes duplicate libraries from the link line,
# saving only the last one.
#
#list(REVERSE ${package}_LIBRARIES)
#list(REMOVE_DUPLICATES ${package}_LIBRARIES)
#list(REVERSE ${package}_LIBRARIES)

# Get the other lflags
#rosbuild_invoke_rospack(${package} ${package} temp libs-only-other)
#_rosbuild_list_to_string(${package}_LDFLAGS_OTHER "${${package}_temp}")
#message("${package} other ldflags: ${${package}_LDFLAGS_OTHER}")

# build introspection for each message in ${package}
message(STATUS "Building message introspection for ${package}")
unset(SOURCES)
unset(DECLARE_MESSAGES)
unset(ADD_MESSAGES)

foreach(message ${MSG_FILES})
  string(REGEX REPLACE "\\.msg$" "" message ${message})
  #message(STATUS "Found message ${package}/${message}")

  #file(READ "${${package}_PACKAGE_PATH}/msg/${message}.msg" definition)
  #string(REGEX REPLACE "\n" " \\\\\\n" definition "${definition}")
  execute_process(COMMAND ${ROSCPP_INTROSPECTION_PACKAGE_PATH}/scripts/get_fields.pl ${${package}_PACKAGE_PATH}/msg/${message}.msg --declaration OUTPUT_VARIABLE DECLARE_FIELDS)
  execute_process(COMMAND ${ROSCPP_INTROSPECTION_PACKAGE_PATH}/scripts/get_fields.pl ${${package}_PACKAGE_PATH}/msg/${message}.msg --constructor OUTPUT_VARIABLE ADD_FIELDS)

  set(header "${${package}_PACKAGE_PATH}/msg_gen/cpp/include/${package}/${message}.h")
  if(NOT EXISTS "${header}")
    message(SEND_ERROR "Could not find cpp header ${package}/${message}.h. Please make sure that package ${pkg} is built or include it in your manifest.")
  endif(NOT EXISTS "${header}")

  configure_file(introspect_message.cpp.in "${CMAKE_CURRENT_BINARY_DIR}/introspect_${message}.cpp" @ONLY)
  list(APPEND SOURCES "${CMAKE_CURRENT_BINARY_DIR}/introspect_${message}.cpp")

  set(DECLARE_MESSAGES "${DECLARE_MESSAGES}    namespace ${message} { MessagePtr createIntrospection(const Package&); }\n")
  set(ADD_MESSAGES     "${ADD_MESSAGES}    add(${message}::createIntrospection(*this));\n")
endforeach()

set(CMAKE_BUILD_TYPE MinSizeRel)
configure_file(introspect_package.cpp.in "${CMAKE_CURRENT_BINARY_DIR}/introspect_package.cpp" @ONLY)
rosbuild_add_library(roscpp_introspection_${package} ${SOURCES} "${CMAKE_CURRENT_BINARY_DIR}/introspect_package.cpp")
set_target_properties(roscpp_introspection_${package} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib/introspection"
)

#rosbuild_add_compile_flags(roscpp_introspection_${package} ${${package}_CFLAGS_OTHER})
#rosbuild_add_link_flags(roscpp_introspection_${package} ${${package}_LDFLAGS_OTHER})
#target_link_libraries(roscpp_introspection_${package} ${${package}_LIBRARIES})
