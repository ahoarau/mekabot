# - Try to find HOLOMNI
# Once done this will define
#  HOLOMNI_FOUND - System has HOLOMNI
#  HOLOMNI_INCLUDE_DIRS - The HOLOMNI include directories
#  HOLOMNI_LIBRARIES - The libraries needed to use HOLOMNI
#  HOLOMNI_DEFINITIONS - Compiler switches required for using HOLOMNI

find_package(PkgConfig)
pkg_check_modules(HOLOMNI QUIET libholomni_pcv)
set(HOLOMNI_DEFINITIONS ${HOLOMNI_CFLAGS_OTHER})

find_path(HOLOMNI_INCLUDE_DIR holomni_pcv/Caster.h
          HINTS ${HOLOMNI_INCLUDEDIR} ${HOLOMNI_INCLUDE_DIRS}
          PATH_SUFFIXES HOLOMNI )

find_library(HOLOMNI_LIBRARY NAMES holomni_pcv holomni HOLOMNI
             HINTS ${HOLOMNI_LIBDIR} ${HOLOMNI_LIBRARY_DIRS} )

set(HOLOMNI_LIBRARIES ${HOLOMNI_LIBRARY} )
set(HOLOMNI_INCLUDE_DIRS ${HOLOMNI_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set HOLOMNI_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(HOLOMNI  DEFAULT_MSG
                                  HOLOMNI_LIBRARY HOLOMNI_INCLUDE_DIR)

mark_as_advanced(HOLOMNI_INCLUDE_DIR HOLOMNI_LIBRARY )
