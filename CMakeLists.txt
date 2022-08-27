cmake_minimum_required(VERSION 2.6)

PROJECT(PiRaTe)

include(GNUInstallDirs)

#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_BUILD_TYPE Debug)

ADD_DEFINITIONS(-std=gnu++17)
ADD_DEFINITIONS(-Wall)
ADD_DEFINITIONS(-Wno-psabi)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(ratsche)
add_subdirectory(pirt)
add_subdirectory(macros)
add_subdirectory(config)

