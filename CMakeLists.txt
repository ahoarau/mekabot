cmake_minimum_required(VERSION 2.8)
project(mekabot)

set(M3RT_SRC ${CMAKE_SOURCE_DIR}/m3core/src) # Used by FindM3rt.cmake to locate
                                             # the core files before their
                                             # installation on the first build.
set(CMAKE_MODULE_PATH 
    "${CMAKE_SOURCE_DIR}/meka-cmake-modules/Modules/"
    ${CMAKE_MODULE_PATH})

set(PACKAGE_VERSION 1.8.0)
add_subdirectory(m3core)
add_subdirectory(m3meka)

