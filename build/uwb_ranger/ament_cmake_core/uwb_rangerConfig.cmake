# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_uwb_ranger_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED uwb_ranger_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(uwb_ranger_FOUND FALSE)
  elseif(NOT uwb_ranger_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(uwb_ranger_FOUND FALSE)
  endif()
  return()
endif()
set(_uwb_ranger_CONFIG_INCLUDED TRUE)

# output package information
if(NOT uwb_ranger_FIND_QUIETLY)
  message(STATUS "Found uwb_ranger: 6.1.0 (${uwb_ranger_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'uwb_ranger' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT uwb_ranger_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(uwb_ranger_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${uwb_ranger_DIR}/${_extra}")
endforeach()
