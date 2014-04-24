
# deduce the libraries suffix from the options
set(FIND_M3_LIB_SUFFIX "")

# find the M3 include directory
find_path(M3_ROOT m3
	PATH_SUFFIXES include
		PATHS
		/usr/local/
		/usr)

set(M3_ROOT ${M3_ROOT}/m3/)


# find the requested modules
set(M3_FOUND TRUE) # will be set to false if one of the required modules is not found
set(FIND_M3_LIB_PATHS
		/usr/local
		/usr)

foreach(FIND_M3_COMPONENT ${M3_FIND_COMPONENTS})
    string(TOLOWER ${FIND_M3_COMPONENT} FIND_M3_COMPONENT_LOWER)
    #Taking care of the shared_mem exception TODO: Correct that
    if(${FIND_M3_COMPONENT_LOWER} MATCHES "shared_mem")
        set(FIND_M3_COMPONENT_LOWER "shm")
    endif()
    set(FIND_M3_COMPONENT_NAME m3${FIND_M3_COMPONENT_LOWER}${FIND_M3_LIB_SUFFIX})
    set(M3_INCLUDE_DIR ${M3_INCLUDE_DIR} ${M3_ROOT}${FIND_M3_COMPONENT_LOWER})
    find_library(M3_${FIND_M3_COMPONENT} NAMES ${FIND_M3_COMPONENT_NAME} PATHS ${FIND_M3_LIB_PATHS} PATH_SUFFIXES lib)
    set(M3_LIBRARIES ${M3_LIBRARIES} ${M3_${FIND_M3_COMPONENT}})
endforeach()
message("M3 Libraries : ${M3_LIBRARIES}")




