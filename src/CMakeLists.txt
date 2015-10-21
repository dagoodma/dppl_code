# CMakeLists
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#ADD_EXECUTABLE( nearestNeighborDtsp nearestNeighborDtsp.cpp Dubins.cpp )
#ADD_EXECUTABLE( computeDubinsDistancesAtsp computeDubinsDistancesAtsp.cpp Dubins.cpp TSPLib.cpp )
#ADD_EXECUTABLE( alternatingDtsp alternatingDtsp.cpp TSPLib.cpp Dubins.cpp )
#ADD_EXECUTABLE( randomizedDtsp randomizedDtsp.cpp TSPLib.cpp Dubins.cpp )
#
#TARGET_LINK_LIBRARIES( nearestNeighborDtsp OGDF COIN )
#TARGET_LINK_LIBRARIES( computeDubinsDistancesAtsp OGDF COIN )
#TARGET_LINK_LIBRARIES( optimalEuclideanTSP OGDF COIN LKH )
#TARGET_LINK_LIBRARIES( alternatingDtsp OGDF COIN LKH )
#TARGET_LINK_LIBRARIES( randomizedDtsp OGDF COIN LKH )

################################
# DPP Library
################################
add_library(DPP STATIC
    dpp/basic/basic.cpp
    dpp/basic/Logger.cpp
    dpp/basic/Path.cpp
    dpp/basic/FileIO.cpp
    dpp/planalg/Algorithm.cpp
    dpp/planalg/RandomizedDtsp.cpp
    dpp/planalg/NearestNeighborDtsp.cpp
    dpp/planalg/AlternatingDtsp.cpp
    dpp/planalg/FieldTracksCpp.cpp
    dpp/planner/PathPlanner.cpp
    dpp/planner/DubinsVehiclePathPlanner.cpp
    dpp/planner/DubinsSensorPathPlanner.cpp
)

TARGET_LINK_LIBRARIES( DPP OGDF COIN )

################################
# Binaries
################################

# Solve DTSP binary
ADD_EXECUTABLE( solveDtsp solveDtsp.cpp )

TARGET_LINK_LIBRARIES( solveDtsp OGDF COIN DPP )

# Solve CPP as DTSP binary
ADD_EXECUTABLE( solveCppAsDtsp solveCppAsDtsp.cpp )

TARGET_LINK_LIBRARIES( solveCppAsDtsp OGDF COIN DPP )

