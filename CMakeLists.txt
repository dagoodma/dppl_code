# CMakeLists
cmake_minimum_required(VERSION 3.1)
project (dubinsAreaCoverage CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

enable_testing()

################################
# OGDF
################################
find_library(OGDF ogdf)
find_library(COIN coin)

include_directories( ${CMAKE_SOURCE_DIR}/lib/ogdf/include )
link_directories( ${CMAKE_SOURCE_DIR}/lib/ogdf/_debug )

################################
# Extra libraries
################################
set(LKH_EXECUTABLE_PATH "LKH") # uses relative path or $PATH

# Header-only libraries are include only
include_directories( ${CMAKE_SOURCE_DIR}/lib/eigen-eigen-bdd17ee3b1b3 )
include_directories( ${CMAKE_SOURCE_DIR}/lib/cxxopts/src )
include_directories( ${CMAKE_SOURCE_DIR}/lib/stacktrace )

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

endif ( CMAKE_BUILD_TYPE MATCHES Debug )

################################
# CMake Project Subdirectories
################################
## Add subdirectories
add_subdirectory( src )
add_subdirectory( test )

