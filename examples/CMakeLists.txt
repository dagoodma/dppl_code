# CMakeLists
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

################################
# Binaries
################################

# Solve DTSP example binary
ADD_EXECUTABLE( solveDtsp_example solveDtsp_example.cpp )
TARGET_LINK_LIBRARIES( solveDtsp_example ${OGDF_LIBRARIES} DPP )

# Solve DTSP simplified example binary
ADD_EXECUTABLE( solveDtspSimple_example solveDtspSimple_example.cpp )
TARGET_LINK_LIBRARIES( solveDtspSimple_example ${OGDF_LIBRARIES} DPP )

