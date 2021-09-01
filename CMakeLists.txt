cmake_minimum_required(VERSION 2.6)

PROJECT(PiRaTe)

include(GNUInstallDirs)

#SET(CMAKE_VERBOSE_MAKEFILE ON)

#The following lines are required to use Dart
#ENABLE_TESTING()
#INCLUDE(Dart)

SET(CMAKE_BUILD_TYPE Debug)

#ADD_DEFINITIONS(-mno-sse2)
ADD_DEFINITIONS(-std=gnu++17)
#ADD_DEFINITIONS(-ffast-math)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(ratsche)
add_subdirectory(pirt)
add_subdirectory(macros)
add_subdirectory(config)

