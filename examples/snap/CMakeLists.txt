# CMakeLists
cmake_minimum_required(VERSION 3.1)
project (EXAMPLES)

find_library(SNAP snap
    PATHS ../lib/snap/snap-core)
#find_library(OGDF ogdf
#    PATHS   ../lib/ogdf/_debug)

## OGDF Examples
#FIND
#FIND_PACKAGE( OGDF REQUIRED )
# set the path to the library folder
link_directories(/usr/local/lib)

# link the libraries to the executable

#INCLUDE_DIRECTORIES( /usr/local/include/ogdf )
#INCLUDE_DIRECTORIES( /usr/local/include/coin )

INCLUDE_DIRECTORIES( ../lib/snap/snap-core )

## c0x
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#else()
        #message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()


# #SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} -g" )

## Build and Link Executables

ADD_EXECUTABLE( directed_graph1 directed_graph1.cpp )

TARGET_LINK_LIBRARIES( directed_graph1 snap )


#ADD_EXECUTABLE( manual manual.cpp )
#ADD_EXECUTABLE( random random.cpp )
#ADD_EXECUTABLE( sierpinski sierpinski.cpp )

#TARGET_LINK_LIBRARIES( manual OGDF )
#TARGET_LINK_LIBRARIES( random OGDF )
#TARGET_LINK_LIBRARIES( sierpinski OGDF )

## Boost Examples
#FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
#INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
#
##ADD_EXECUTABLE( dgaTest directed_graph_test.cpp )
#ADD_EXECUTABLE( graphVizTest write_graphviz.cpp )
#ADD_EXECUTABLE( graphVizReadTest read_graphviz.cpp )
#
#TARGET_LINK_LIBRARIES( graphVizTest ${Boost_LIBRARIES} )
##TARGET_LINK_LIBRARIES( dgaTest ${Boost_LIBRARIES} )
#TARGET_LINK_LIBRARIES( graphVizReadTest ${Boost_LIBRARIES} )

# the compiler used for C files 
MESSAGE( STATUS "CMAKE_C_COMPILER: " ${CMAKE_C_COMPILER} )

# the compiler used for C++ files 
MESSAGE( STATUS "CMAKE_CXX_COMPILER: " ${CMAKE_CXX_COMPILER} )

# if the compiler is a variant of gcc, this should be set to 1 
MESSAGE( STATUS "CMAKE_COMPILER_IS_GNUCC: " ${CMAKE_COMPILER_IS_GNUCC} )

# if the compiler is a variant of g++, this should be set to 1 
MESSAGE( STATUS "CMAKE_COMPILER_IS_GNUCXX : " ${CMAKE_COMPILER_IS_GNUCXX} )
