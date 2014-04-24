
# deduce the libraries suffix from the options
set(FIND_M3RT_LIB_SUFFIX "")

# find the M3RT include directory
find_path(M3RT_ROOT m3rt
	PATH_SUFFIXES include
		PATHS
		/usr/local/
		/usr)

set(M3RT_ROOT ${M3RT_ROOT}/m3rt/)

# find the requested modules
set(M3RT_FOUND TRUE) # will be set to false if one of the required modules is not found
set(FIND_M3RT_LIB_PATHS
		/usr/local
		/usr)

set(FIND_M3RT_COMPONENT "base")
string(TOLOWER ${FIND_M3RT_COMPONENT} FIND_M3RT_COMPONENT_LOWER)
set(FIND_M3RT_COMPONENT_NAME m3${FIND_M3RT_COMPONENT_LOWER}${FIND_M3RT_LIB_SUFFIX})

set(M3RT_INCLUDE_DIR ${M3RT_INCLUDE_DIR} ${M3RT_ROOT}${FIND_M3RT_COMPONENT_LOWER})
find_library(M3RT_${FIND_M3RT_COMPONENT} NAMES ${FIND_M3RT_COMPONENT_NAME} PATHS ${FIND_M3RT_LIB_PATHS} PATH_SUFFIXES lib)

set(M3RT_LIBRARIES ${M3RT_LIBRARIES} ${M3RT_${FIND_M3RT_COMPONENT}})
message("M3RT Libraries : ${M3RT_LIBRARIES}")
