# - Try to find OGDF and COIN
# TODO fixme
# Once done this will define
#  OGDF_FOUND - System has ogdf
#  OGDF_INCLUDE_DIRS - The ogdf include directories
#  OGDF_LIBRARIES - The libraries needed to use ogdf
#  COIN_FOUND - System has coin
#  COIN_INCLUDE_DIRS - The coin include directories
#  COIN_LIBRARIES - The libraries needed to use coin

# Find includes for ogdf and coin
find_path(OGDF_INCLUDE_DIR ogdf/basic/basic.h coin/Coin_C_defines.h
	PATHS: "lib/ogdf/include" )

# Find OGDF lib
find_library(OGDF_LIBRARY_DEBUG NAMES ogdf 
	HINTS lib/ogdf
	PATH_SUFFIXES _debug )
find_library(OGDF_LIBRARY_RELEASE NAMES ogdf 
	HINTS lib/ogdf
	PATH_SUFFIXES _release )

# Find COIN lib
find_library(COIN_LIBRARY_DEBUG NAMES coin
	HINTS lib/ogdf
	PATH_SUFFIXES _debug )
find_library(COIN_LIBRARY_RELEASE NAMES coin
	HINTS lib/ogdf
	PATH_SUFFIXES _release )

# Set debug and release versions
set( OGDF_LIBRARY debug     ${OGDF_LIBRARY_DEBUG}
                  optimized ${OGDF_LIBRARY_RELEASE} )

set( COIN_LIBRARY debug     ${COIN_LIBRARY_DEBUG}
                  optimized ${COIN_LIBRARY_RELEASE} )

message(STATUS "OGDF lib: " ${OGDF_LIBRARY})
message(STATUS "COin lib: " ${COIN_LIBRARY})

# set variables
set(OGDF_LIBRARIES ${OGDF_LIBRARY} )
set(OGDF_LIBRARIES ${OGDF_LIBRARIES} ${COIN_LIBRARY} )

set(OGDF_INCLUDE_DIRS ${OGDF_INCLUDE_DIR} )

message(STATUS "Here with: " ${OGDF_LIBRARIES})
message(STATUS "Here with: " ${OGDF_INCLUDE_DIRS})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set OGDF_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Ogdf  DEFAULT_MSG
                                  OGDF_LIBRARY OGDF_INCLUDE_DIR)

mark_as_advanced(OGDF_INCLUDE_DIR OGDF_LIBRARY COIN_LIBRARY )
