# CMakeLists
cmake_minimum_required(VERSION 3.1)
project (dubinsAreaCoverage CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

enable_testing()

################################
# OGDF
################################
# FIXME using hardcoded ogdf variables
#find_package(Ogdf REQUIRED)
#include_directories( ${OGDF_INCLUDE_DIRS} )

set(OGDF_SOURCE_DIR ${CMAKE_SOURCE_DIR}/lib/ogdf )

find_library(OGDF ogdf)
find_library(COIN coin)


# Look for include directories
find_path(OGDF_INCLUDE_DIR ogdf/basic/basic.h coin/Coin_C_defines.h
    PATHS: ${OGDF_SOURCE_DIR}/include )

if (OGDF_INCLUDE_DIR STREQUAL "OGDF_INCLUDE_DIR-NOTFOUND")
    message(FATAL_ERROR "OGDF include directory could not be found!")
endif()

include_directories( ${OGDF_INCLUDE_DIR} )

# Look for libraries
find_path(OGDF_LIBRARY_DIR libCOIN.a libOGDF.a
    PATHS: ${OGDF_SOURCE_DIR}/_debug ${OGDF_SOURCE_DIR}/_release )

if (OGDF_LIBRARY_DIR STREQUAL "OGDF_LIBRARY_DIR-NOTFOUND")
    message(FATAL_ERROR "OGDF libraries could not be found!")
endif()

# Add libraries to link directories
link_directories( ${OGDF_SOURCE_DIR}/_debug )
if ( CMAKE_BUILD_TYPE MATCHES Debug )
    link_directories( ${OGDF_SOURCE_DIR}/_debug )
else()
    link_directories( ${OGDF_SOURCE_DIR}/_release )
endif()

set( OGDF_LIBRARIES OGDF COIN )

################################
# LKH Binary
################################
find_program(LKH_EXECUTABLE LKH . PATHS: lib/LKH lib/LKH-2.0.7 )
message("-- Found LKH binary: " ${LKH_EXECUTABLE})

if ( LKH_EXECUTABLE STREQUAL "LKH_EXECUTABLE-NOTFOUND" )
    message(FATAL_ERROR "LKH executable could not found!")
else()
    set( LKH_EXECUTABLE_FLAG "-DLKH_EXECUTABLE='\"${LKH_EXECUTABLE}\"'")
    set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${LKH_EXECUTABLE_FLAG}"  )
endif()

################################
# Dubins-Curves Libraries
################################

link_directories( ${CMAKE_SOURCE_DIR}/lib/dubins-curves )
include_directories( ${CMAKE_SOURCE_DIR}/lib/dubins-curves/include )

################################
# Eigen and others
################################
# Header-only libraries are include only
find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package(Cxxopts REQUIRED)
include_directories( ${CXXOPTS_INCLUDE_DIR} )

find_package(Stacktrace REQUIRED)
include_directories( ${STACKTRACE_INCLUDE_DIR} )

################################
# Dubins Path Planner
################################
# DPP library includes
include_directories( ${CMAKE_SOURCE_DIR}/include )
include_directories( ${CMAKE_SOURCE_DIR}/include/dpp )
include_directories( ${CMAKE_SOURCE_DIR}/include/dpp/basic )
include_directories( ${CMAKE_SOURCE_DIR}/include/dpp/planner )
include_directories( ${CMAKE_SOURCE_DIR}/include/dpp/planalg )

################################
# Settings and Compiler Flags
################################
# Set c++11 (c++0x)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Make binaries output in the same directory
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

## Compiler flags
# OSX, TODO add if statement
add_definitions("-fno-pie")

# Debug flags
if ( CMAKE_BUILD_TYPE MATCHES Debug )
    message( "Compiling DPP in debug mode." )
    set(GCC_DPP_DEBUG_COMPILE_FLAGS "-DDPP_DEBUG")
    set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GCC_DPP_DEBUG_COMPILE_FLAGS}" )

    ## Print debug information about compiler
    # the compiler used for C files 
    MESSAGE( STATUS "CMAKE_C_COMPILER: " ${CMAKE_C_COMPILER} )

    # the compiler used for C++ files 
    MESSAGE( STATUS "CMAKE_CXX_COMPILER: " ${CMAKE_CXX_COMPILER} )

    # if the compiler is a variant of gcc, this should be set to 1 
    MESSAGE( STATUS "CMAKE_COMPILER_IS_GNUCC: " ${CMAKE_COMPILER_IS_GNUCC} )

    # if the compiler is a variant of g++, this should be set to 1 
    MESSAGE( STATUS "CMAKE_COMPILER_IS_GNUCXX : " ${CMAKE_COMPILER_IS_GNUCXX} )

    MESSAGE( STATUS "CXX Compiler flags: " ${CMAKE_CXX_FLAGS} )

    MESSAGE( STATUS "Include paths:")

    get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
    foreach(dir ${dirs})
      message(STATUS "dir='${dir}'")
    endforeach()

endif ( CMAKE_BUILD_TYPE MATCHES Debug )

################################
# CMake Project Subdirectories
################################
## Add subdirectories
add_subdirectory( src )
add_subdirectory( test )
add_subdirectory( examples )

