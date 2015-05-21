# CMakeLists
cmake_minimum_required(VERSION 3.1)
project (EXAMPLES)

FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

ADD_EXECUTABLE( dgaTest directed_graph_test.cpp )
ADD_EXECUTABLE( graphVizTest write_graphviz.cpp )

TARGET_LINK_LIBRARIES( graphVizTest ${Boost_LIBRARIES} )
TARGET_LINK_LIBRARIES( dgaTest ${Boost_LIBRARIES} )
