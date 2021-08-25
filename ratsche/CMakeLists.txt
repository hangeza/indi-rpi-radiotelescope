cmake_minimum_required(VERSION 2.6)

#Change PROJECT_NAME to the name of your project
PROJECT(Ratsche)

#SET(CMAKE_VERBOSE_MAKEFILE ON)

#include some macros from another file...
#include(${CMAKE_SOURCE_DIR}/cmake/Modules/Macros.cmake)
#include(${CMAKE_SOURCE_DIR}/cmake/Modules/FindFFTW3.cmake)
#include(${CMAKE_SOURCE_DIR}/cmake/Modules/FindROOT.cmake)

#include some macros from another file...
#INCLUDE(${CMAKE_SOURCE_DIR}/cmake/CMakeMacros.txt)

#INCLUDE(CMakeMacros.txt)
#INCLUDE (CheckIncludeFiles)

#The following lines are required to use Dart
ENABLE_TESTING()
#INCLUDE(Dart)

SET(CMAKE_BUILD_TYPE Debug)

## Locate additional CMake find scripts

#SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

#find_package(FFTW3 REQUIRED)
#find_package(CFITSIO REQUIRED)
#find_package(Boost REQUIRED)

#ADD_DEFINITIONS(-mno-sse2)
ADD_DEFINITIONS(-std=gnu++17)
#ADD_DEFINITIONS(-ffast-math)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

#CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)

#Set any libraries that your project depends on.
#examples: ITKCommon, VTKRendering, etc
#SET(Libraries ...)
#include(CMakeCommon)

# tell cmake to build our executable

ADD_EXECUTABLE(ratsche 
	ratsche_main.cpp
	basic.cpp
	rttask.cpp
	time.cpp
	astro.cpp
)

TARGET_LINK_LIBRARIES(ratsche
 pthread
)

