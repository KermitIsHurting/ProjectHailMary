# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_cuas_fusion_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED cuas_fusion_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(cuas_fusion_FOUND FALSE)
  elseif(NOT cuas_fusion_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(cuas_fusion_FOUND FALSE)
  endif()
  return()
endif()
set(_cuas_fusion_CONFIG_INCLUDED TRUE)

# output package information
if(NOT cuas_fusion_FIND_QUIETLY)
  message(STATUS "Found cuas_fusion: 0.1.0 (${cuas_fusion_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'cuas_fusion' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${cuas_fusion_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(cuas_fusion_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${cuas_fusion_DIR}/${_extra}")
endforeach()
