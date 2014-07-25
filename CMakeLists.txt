cmake_minimum_required(VERSION 2.8)
project(mekabot)

set(CMAKE_MODULE_PATH 
    "${CMAKE_SOURCE_DIR}/meka-cmake-modules/Modules/"
    ${CMAKE_MODULE_PATH}
)

# Option for Rtai, then check if available
OPTION(RTAI "Enable/Disable RTAI (used for python proxy users only)" ON)

if(RTAI)
    find_package(RTAI)
endif()

if(RTAI AND NOT RTAI_FOUND)
    set(RTAI OFF)
    message(WARNING "Rtai not present, only generating python")
endif()

# Compile in c++11
OPTION(C++11 "Use c++11 " 0)

if(C++11)
	set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
	message("-- Using c++11")
endif(C++11)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#set(M3RT_SRC ${PROJECT_SOURCE_DIR}/m3core/src)
# Include dirs to look for
set(M3RT_SRC ${PROJECT_SOURCE_DIR}/m3core/src)
set(M3_SRC ${PROJECT_SOURCE_DIR}/m3meka/src)

include_directories(
    ${CMAKE_BINARY_DIR}/m3core/src # Include this directory for the generated protobufs headers
    ${CMAKE_BINARY_DIR}/m3core/src/m3rt/base 
)

add_subdirectory(m3core)
add_subdirectory(m3meka)
if(RTAI)
  add_subdirectory(holomni_pcv)
endif(RTAI)

