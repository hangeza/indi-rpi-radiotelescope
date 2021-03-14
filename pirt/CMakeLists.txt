# define the project name
project(indi-pirt C CXX)
cmake_minimum_required(VERSION 2.8)

include(GNUInstallDirs)

# add our cmake_modules folder
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# find our required packages
find_package(INDI REQUIRED)
find_package(Nova REQUIRED)
find_package(ZLIB REQUIRED)
find_package(GSL REQUIRED)
#find_library(LIBGPIOD gpiod REQUIRED)
find_library(PIGPIOD_IF2 pigpiod_if2 REQUIRED)
find_library(RT rt REQUIRED)

#reminder what we installed to get everything running:
# pigpiod libpigpio-dev libpigpiod-if-dev

# these will be used to set the version number in config.h and our driver's xml file
set(CDRIVER_VERSION_MAJOR 0)
set(CDRIVER_VERSION_MINOR 1)

# do the replacement in the config.h
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

# do the replacement in the driver's xml file
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_pirt.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_pirt.xml
)

# set our include directories to look for header files
include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${NOVA_INCLUDE_DIR})
include_directories( ${EV_INCLUDE_DIR})

include(CMakeCommon)

# tell cmake to build our executable
add_executable(
    indi_pirt
	axis.cpp
	gpioif.cpp
	encoder.cpp
	motordriver.cpp
	i2cdevice.cpp
	ads1115.cpp
    pirt.cpp
)

add_executable(
    encodertest
	encodertest.cpp
	gpioif.cpp
	encoder.cpp
)


# and link it to these libraries
target_link_libraries(
    indi_pirt
    ${INDI_LIBRARIES}
    ${NOVA_LIBRARIES}
    ${GSL_LIBRARIES}
    pigpiod_if2
    rt
    pthread
)

target_link_libraries(
    encodertest
    pigpiod_if2
    rt
    pthread
)

# tell cmake where to install our executable
install(TARGETS indi_pirt RUNTIME DESTINATION bin)

# and where to put the driver's xml file.
install(
    FILES
    ${CMAKE_CURRENT_BINARY_DIR}/indi_pirt.xml
    DESTINATION ${INDI_DATA_DIR}
)
