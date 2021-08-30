# - Try to find M
# Once done this will define
#
#  M_FOUND - system has M
#  M_INCLUDE_DIR - the M include directory
#  M_LIB - Link these to use M
#  M_VERSION_STRING - Human readable version number of m
#  M_VERSION_MAJOR  - Major version number of m
#  M_VERSION_MINOR  - Minor version number of m

# Copyright (c) 2017, Ilia Platone, <info@iliaplatone.com>
# Based on FindLibfacile by Carsten Niehaus, <cniehaus@gmx.de>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if (M_LIB)

  # in cache already
  set(M_FOUND TRUE)
  message(STATUS "Found M: ${M_LIB}")


else (M_LIB)

  find_library(M_LIB NAMES m
    PATHS
    ${_obLinkDir}
    ${GNUWIN32_DIR}/lib
    /usr/local/lib
    HINTS ${CMAKE_C_IMPLICIT_LINK_DIRECTORIES}
  )

  if(M_LIB)
    set(M_FOUND TRUE)
  else (M_LIB)
    set(M_FOUND FALSE)
  endif(M_LIB)


  if (M_FOUND)
    if (NOT M_FIND_QUIETLY)
      message(STATUS "Found M: ${M_LIB}")
    endif (NOT M_FIND_QUIETLY)
  else (M_FOUND)
    if (M_FIND_REQUIRED)
      message(FATAL_ERROR "M not found. Please install libm-dev")
    endif (M_FIND_REQUIRED)
  endif (M_FOUND)

  mark_as_advanced(M_LIB)
  
endif (M_LIB)
