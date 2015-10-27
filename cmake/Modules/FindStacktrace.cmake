# - Try to find stacktrace
# Once done this will define
#  STACKTRACE_FOUND - System has cxxopts
#  STACKTRACE_INCLUDE_DIRS - The cxxopts include directories

find_path(STACKTRACE_INCLUDE_DIR stacktrace.h
	PATHS: "lib/stacktrace" )

set(STACKTRACE_INCLUDE_DIRS ${STACKTRACE_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CXXOPTS_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Stacktrace DEFAULT_MSG
                                  STACKTRACE_INCLUDE_DIR)

mark_as_advanced(STACKTRACE_INCLUDE_DIR)