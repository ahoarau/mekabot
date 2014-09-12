cmake_minimum_required(VERSION 2.8)
project(mekabot)

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

add_subdirectory(m3core)
add_subdirectory(m3meka)

find_package(Holomni)
if(NOT HOLOMNI_FOUND AND RTAI)
  add_subdirectory(holomni_pcv)
endif()

