# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_iq_gnc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED iq_gnc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(iq_gnc_FOUND FALSE)
  elseif(NOT iq_gnc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(iq_gnc_FOUND FALSE)
  endif()
  return()
endif()
set(_iq_gnc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT iq_gnc_FIND_QUIETLY)
  message(STATUS "Found iq_gnc: 0.0.0 (${iq_gnc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'iq_gnc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${iq_gnc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(iq_gnc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${iq_gnc_DIR}/${_extra}")
endforeach()
