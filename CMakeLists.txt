cmake_minimum_required(VERSION 2.8...4.0)

project(ahp_gt C CXX)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

include(GNUInstallDirs)

find_package(Threads REQUIRED)

set(LIB_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(AHP_GT_VERSION_MAJOR 2)
set(AHP_GT_VERSION_MINOR 0)
set(AHP_GT_VERSION_RELEASE 1)
set(AHPGT_VERSION "${AHP_GT_VERSION_MAJOR}.${AHP_GT_VERSION_MINOR}.${AHP_GT_VERSION_RELEASE}")
set(AHP_GT_VERSION 0x${AHP_GT_VERSION_MAJOR}${AHP_GT_VERSION_MINOR}${AHP_GT_VERSION_RELEASE})
set(AHPGT_SOVERSION ${AHP_GT_VERSION_MAJOR})

add_definitions(-fno-permissive)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/ahp_gt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/rs232.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/serial.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/impl/list_ports/list_ports_win.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/impl/list_ports/list_ports_osx.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/impl/list_ports/list_ports_linux.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/impl/unix.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/impl/win.cc
   )

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/brew.sh.cmake ${CMAKE_CURRENT_BINARY_DIR}/brew.sh )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/99-libahp-gt.rules.cmake ${CMAKE_CURRENT_BINARY_DIR}/99-libahp-gt.rules )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ahp_gt.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/ahp_gt.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.cmake ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/FindAHPGT.cmake ${CMAKE_CURRENT_BINARY_DIR}/FindAHPGT.cmake )

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_INCLUDE_PATH})
link_directories(${CMAKE_LIBRARY_PATH})

execute_process (COMMAND doxygen ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

add_library(ahp_gt SHARED ${SOURCES} ${PLATFORM_SOURCES})

if(WIN32)
    target_link_libraries(ahp_gt ws2_32 setupapi cfgmgr32)
else(WIN32)
   set_target_properties(ahp_gt PROPERTIES VERSION ${AHPGT_VERSION} SOVERSION ${AHPGT_SOVERSION})
   install(FILES ${CMAKE_CURRENT_BINARY_DIR}/99-libahp-gt.rules DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/udev/rules.d)
endif(WIN32)

target_link_libraries(ahp_gt ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS ahp_gt LIBRARY DESTINATION ${LIB_INSTALL_DIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ahp_gt.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/ahp)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/FindAHPGT.cmake DESTINATION "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules")
