# - Try to find cxxopts
# Once done this will define
#  CXXOPTS_FOUND - System has cxxopts
#  CXXOPTS_INCLUDE_DIRS - The cxxopts include directories

find_path(CXXOPTS_INCLUDE_DIR cxxopts.hpp
	PATHS: "lib/cxxopts/src" )

set(CXXOPTS_INCLUDE_DIRS ${CXXOPTS_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CXXOPTS_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Cxxopts DEFAULT_MSG
                                  CXXOPTS_INCLUDE_DIR)

mark_as_advanced(CXXOPTS_INCLUDE_DIR)