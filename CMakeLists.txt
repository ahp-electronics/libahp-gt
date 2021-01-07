cmake_minimum_required(VERSION 2.6)

project(ahp_gt C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
include(GNUInstallDirs)

set(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(AHPGT_VERSION "1.0.0")
set(AHPGT_SOVERSION "1")

find_package(M REQUIRED)

set(SOURCES
    ${CMAKE_SOURCE_DIR}/rs232.c
    ${CMAKE_SOURCE_DIR}/ahp_gt.c
   )

include_directories(${CMAKE_INCLUDE_PATH})
link_directories(${CMAKE_LIBRARY_PATH})

add_library(ahp_gt SHARED ${SOURCES})
set_target_properties(ahp_gt PROPERTIES VERSION ${AHPGT_VERSION} SOVERSION ${AHPGT_SOVERSION})

install(TARGETS ahp_gt LIBRARY DESTINATION ${LIB_INSTALL_DIR})
install(FILES ahp_gt.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ahp)
