# CMakeLists
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#ADD_EXECUTABLE( nearestNeighborDTSP nearestNeighborDTSP.cpp Dubins.cpp )
#ADD_EXECUTABLE( computeDubinsDistancesATSP computeDubinsDistancesATSP.cpp Dubins.cpp TSPLib.cpp )
#ADD_EXECUTABLE( optimalEuclideanTSP optimalEuclideanTSP.cpp TSPLib.cpp )
#ADD_EXECUTABLE( alternatingDTSP alternatingDTSP.cpp TSPLib.cpp Dubins.cpp )
#ADD_EXECUTABLE( randomizedDTSP randomizedDTSP.cpp TSPLib.cpp Dubins.cpp )
#
#TARGET_LINK_LIBRARIES( nearestNeighborDTSP OGDF COIN )
#TARGET_LINK_LIBRARIES( computeDubinsDistancesATSP OGDF COIN )
#TARGET_LINK_LIBRARIES( optimalEuclideanTSP OGDF COIN LKH )
#TARGET_LINK_LIBRARIES( alternatingDTSP OGDF COIN LKH )
#TARGET_LINK_LIBRARIES( randomizedDTSP OGDF COIN LKH )

# DPP Library
add_library(DPP STATIC
    dpp/basic/basic.cpp
    dpp/basic/Logger.cpp
    dpp/basic/Path.cpp
    dpp/basic/TSPIO.cpp
    dpp/planalg/Algorithm.cpp
    dpp/planalg/RandomizedDTSP.cpp
    dpp/planner/PathPlanner.cpp
    dpp/planner/DubinsVehiclePathPlanner.cpp
)

TARGET_LINK_LIBRARIES( DPP OGDF COIN )

# Solve DTSP binary
ADD_EXECUTABLE( solveDTSP solveDTSP.cpp )

TARGET_LINK_LIBRARIES( solveDTSP OGDF COIN DPP )

