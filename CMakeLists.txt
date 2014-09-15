cmake_minimum_required(VERSION 2.8)
project(mekabot)

## Mekabot Version
SET(MAJOR_VERSION "1")
SET(MINOR_VERSION "9")
SET(PATCH_VERSION "6")
set(M3_VERSION
  ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})
  
  
set(CMAKE_MODULE_PATH 
    "${CMAKE_SOURCE_DIR}/meka-cmake-modules/Modules/"
    ${CMAKE_MODULE_PATH}
)
OPTION(ETHERCAT "Enable/Disable EtherCAT (sim or not)" OFF)

OPTION(VIRTUAL_MEKA "Simulated robot (no hardware)" ON)

# Option for Rtai, then check if available
OPTION(RTAI "Enable/Disable RTAI (used for python proxy users only)" ON)

if(ETHERCAT)
set(VIRTUAL_MEKA 0)
endif()

if(RTAI AND NOT ETHERCAT)
    find_package(RTAI)
elseif(RTAI AND ETHERCAT)
    find_package(RTAI REQUIRED)
endif()

if(RTAI AND NOT RTAI_FOUND)
    set(RTAI OFF)
    message(WARNING "Rtai not present, only generating python")
endif()

# Compile in c++11
OPTION(C++11 "Use c++11 " 0)

if(C++11)
	set(CMAKE_CXX_FLAGS "-std=c++0x")
	message("-- Using c++11")
endif(C++11)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Include dirs to look for
set(M3RT_SRC ${CMAKE_SOURCE_DIR}/m3core/src )
set(M3_SRC ${CMAKE_SOURCE_DIR}/m3meka/src)

# HACK : Include this directory for the generated protobufs headers from m3core
include_directories(
    ${M3RT_SRC}
    ${CMAKE_BINARY_DIR}/m3core/src/m3rt/base 
)



## Enable PPA upload
OPTION(ENABLE_PPA "Enable PPA creation" FALSE)
OPTION(UPLOAD_PPA "Enable PPA upload" FALSE)
OPTION(CHANGELOG_MESSAGE "Set the changelog message" OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

if(NOT ENABLE_PPA)
        add_subdirectory(m3core)
        add_subdirectory(m3meka)
        find_package(Holomni)
        if(NOT HOLOMNI_FOUND AND RTAI)
          add_subdirectory(holomni_pcv)
        endif()
endif()

######## Debian #######

#### Package Generation
execute_process (
  COMMAND          /usr/bin/dpkg --print-architecture
  OUTPUT_VARIABLE  CPACK_DEBIAN_PACKAGE_ARCHITECTURE
  RESULT_VARIABLE  EXECUTE_RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(EXECUTE_RESULT)
  message(STATUS "dpkg not found: No package generation.")
else()
IF(EXISTS "${CMAKE_ROOT}/Modules/CPack.cmake")
INCLUDE(InstallRequiredSystemLibraries)
message(STATUS "Debian architecture: ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

#SET(CPACK_SET_DESTDIR "on")
#SET(CPACK_PACKAGING_INSTALL_PREFIX "/tmp")
SET(CPACK_GENERATOR "DEB")

SET(CPACK_PACKAGE_DESCRIPTION "Holomni PCV : control the holomni wheels")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "This package is used to control the B1 base on the Meka robot")

SET(CPACK_PACKAGE_VENDOR "Meka Robotics LLC, Inria, Ensta ParisTech")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Antoine Hoarau <hoarau.robotics@gmail.com>")
SET(CPACK_PACKAGE_CONTACT "Antoine Hoarau <hoarau.robotics@gmail.com>")
set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/ahoarau/holomni_pcv")
SET(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_MINOR "${MINOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_PATCH "${PATCH_VERSION}")
SET(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${MAJOR_VERSION}.${MINOR_VERSION}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${MAJOR_VERSION}.${MINOR_VERSION}.${CPACK_PACKAGE_VERSION_PATCH}")

set(CPACK_STRIP_FILES TRUE)
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "any") # can be build on any system
#set(PPA_DEBIAN_VERSION "0ubuntu1")
#dependencies for this service menu
set(CPACK_DEBIAN_PACKAGE_BUILD_DEPENDS cmake bzr git pkg-config devscripts libyaml-cpp-dev dh-make libeigen3-dev libprotobuf-dev protobuf-compiler gnuplot-x11 libboost-dev python-dev python-protobuf python-matplotlib python-yaml python-gnuplot python-scipy python-sip-dev python-sip sip-dev swig python-pandas python-sympy python-nose python-numpy)
set(CPACK_DEBIAN_PACKAGE_DEPENDS libeigen3-dev libprotobuf-dev protobuf-compiler gnuplot-x11 libboost-dev python-dev python-protobuf python-matplotlib python-yaml python-gnuplot python-scipy python-sip-dev python-sip sip-dev swig python-pandas python-sympy python-nose python-numpy)

set(CPACK_DISTRIB_TARGET "utopic" "trusty")
 # See ~/.dput.cd
set(DPUT_HOST hoarau-robotics)
set(CPACK_DEBIAN_UPDATE_CHANGELOG TRUE)
#SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
#SET(CPACK_DEBIAN_PACKAGE_SECTION "libraries")
#SET(CPACK_DEBIAN_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
 
#SET(CPACK_COMPONENTS_ALL Libraries ApplicationData)
INCLUDE(CPack)
if(ENABLE_PPA OR UPLOAD_PPA)
    include(UploadPPA)
endif()

ENDIF(EXISTS "${CMAKE_ROOT}/Modules/CPack.cmake")
endif()
