# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_finalproject_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED finalproject_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(finalproject_FOUND FALSE)
  elseif(NOT finalproject_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(finalproject_FOUND FALSE)
  endif()
  return()
endif()
set(_finalproject_CONFIG_INCLUDED TRUE)

# output package information
if(NOT finalproject_FIND_QUIETLY)
  message(STATUS "Found finalproject: 0.0.0 (${finalproject_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'finalproject' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${finalproject_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(finalproject_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${finalproject_DIR}/${_extra}")
endforeach()
