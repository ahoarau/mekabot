cmake_minimum_required(VERSION 2.8)
project(mekabot)


# Compile in c++11
OPTION(C++11 "Use c++11 " 0)

if(C++11)
	set(CMAKE_CXX_FLAGS "-Wall -std=c++11")
	message("-- Using c++11")
endif(C++11)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(MEKABOT_ROOT ${PROJECT_SOURCE_DIR})
set(M3RT_SRC ${PROJECT_SOURCE_DIR}/m3core/src)

include_directories(
${M3RT_SRC}
${CMAKE_BINARY_DIR}/m3core/src
${CMAKE_BINARY_DIR}/m3core/src/m3rt/base	
)
SET(PROTOBUF_IMPORT_DIRS ${M3RT_INCLUDE_DIR} ${M3_INCLUDE_DIR})

set(CMAKE_MODULE_PATH 
    "${CMAKE_SOURCE_DIR}/meka-cmake-modules/Modules/"
    ${CMAKE_MODULE_PATH})

add_subdirectory(m3core)
add_subdirectory(m3meka)

add_subdirectory(holomni_pcv)


