rosbuild_find_ros_package(hector_roscpp_introspection)
set(ROSCPP_INTROSPECTION_PACKAGE_PATH ${hector_roscpp_introspection_PACKAGE_PATH})

set(introspection_package_list "" CACHE INTERNAL "")
mark_as_advanced(introspection_package_list)

include(CMakeParseArguments)
function(introspection_add package)
  cmake_parse_arguments(introspection_add "QUIET RECURSIVE" "" "" ${ARGN})

  # abort if this function has already been called for this package (prevent recursive calls)
  list(FIND introspection_package_list ${package} found)
  if (NOT ${found} EQUAL -1)
    return()
  endif()
  set(introspection_package_list ${introspection_package_list} ${package} CACHE INTERNAL "")

  # recursively call introspection_add for dependent packages
  if(RECURSIVE)
    rosbuild_invoke_rospack(${package} ${package} DEPENDS depends1)
    string(REPLACE "\n" " " ${package}_DEPENDS "${${package}_DEPENDS}")
    separate_arguments(${package}_DEPENDS)
    #message("Package ${package} depends on ${${package}_DEPENDS}")
    foreach(depends ${${package}_DEPENDS})
      introspection_add(${depends} QUIET RECURSIVE)
    endforeach(depends)
  endif(RECURSIVE)

  add_subdirectory(${ROSCPP_INTROSPECTION_PACKAGE_PATH}/src/package roscpp_introspection_${package})
endfunction(introspection_add)
