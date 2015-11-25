# CMakeLists
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

################################
# DPP Library
################################
add_library(DPP STATIC
    dpp/basic/basic.cpp
    dpp/basic/Logger.cpp
    dpp/basic/Path.cpp
    dpp/basic/Field.cpp
    dpp/basic/FieldTrack.cpp
    dpp/basic/FileIO.cpp
    dpp/planalg/Algorithm.cpp
    dpp/planalg/RandomizedDtsp.cpp
    dpp/planalg/NearestNeighborDtsp.cpp
    dpp/planalg/AlternatingDtsp.cpp
    dpp/planalg/BoustrophedonCpp.cpp
    dpp/planner/PathPlanner.cpp
    dpp/planner/DubinsVehiclePathPlanner.cpp
    dpp/planner/DubinsSensorPathPlanner.cpp
    dpp/planner/WaypointSequencePlanner.cpp
    dpp/planner/CoverageWaypointPlanner.cpp
)

TARGET_LINK_LIBRARIES( DPP ${OGDF_LIBRARIES} DUBINSCURVES )

################################
# Binaries
################################

# Solve DTSP binary
ADD_EXECUTABLE( solveDtsp solveDtsp.cpp )
TARGET_LINK_LIBRARIES( solveDtsp ${OGDF_LIBRARIES} DPP DUBINSCURVES )

# Solve CPP as DTSP binary
ADD_EXECUTABLE( solveCppAsDtsp solveCppAsDtsp.cpp )
TARGET_LINK_LIBRARIES( solveCppAsDtsp ${OGDF_LIBRARIES} DPP DUBINSCURVES )

# Solve CPP binary
ADD_EXECUTABLE( solveCpp solveCpp.cpp )
TARGET_LINK_LIBRARIES( solveCpp ${OGDF_LIBRARIES} DPP DUBINSCURVES )

