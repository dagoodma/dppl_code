# CMakeLists

################################
# GTest
################################
# googletest & googlemock
set(GMOCK_DIR "gtest/googlemock"
    CACHE PATH "The path to the GoogleMock test framework.")

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # force this option to ON so that Google Test will use /MD instead of /MT
    # /MD is now the default for Visual Studio, so it should be our default, too
    option(gtest_force_shared_crt
           "Use shared (DLL) run-time lib even when Google Test is built as static lib."
           ON)
elseif (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=1)
endif()

add_subdirectory(${GMOCK_DIR} ${CMAKE_BINARY_DIR}/gmock)
include_directories(SYSTEM ${GMOCK_DIR}/../googletest/include
                           ${GMOCK_DIR}/include)

################################
# Extra Dependencies
################################
# DubinsCurves
link_directories( ${CMAKE_SOURCE_DIR}/lib/dubins-curves )
include_directories( ${CMAKE_SOURCE_DIR}/lib/dubins-curves/include )

# DPP
include_directories( ${CMAKE_SOURCE_DIR}/test )

################################
# Unit Tests
################################

# dpp/basic/Logger
add_executable(Logger_test basic/Logger_test.cpp)
target_link_libraries(Logger_test gmock_main DPP )
add_test(BasicTests Logger_test)

# dpp/basic/Util
add_executable(Util_test basic/Util_test.cpp )
target_link_libraries(Util_test gmock_main OGDF COIN DPP )
add_test(BasicTests Util_test)

# dpp/basic/Path
add_executable(Path_test basic/Path_test.cpp)
target_link_libraries(Path_test gmock_main OGDF COIN DPP DUBINSCURVES)
add_test(BasicTests Path_test)

# dpp/planalg/Field
add_executable(Field_test basic/Field_test.cpp)
target_link_libraries(Field_test gmock_main OGDF COIN DPP)
add_test(PlanalgTests Field_test)

# dpp/planalg/RandomizedDtsp
add_executable(RandomizedDtsp_test planalg/RandomizedDtsp_test.cpp)
target_link_libraries(RandomizedDtsp_test gmock_main OGDF COIN DPP)
add_test(PlanalgTests RandomizedDtsp_test)

# dpp/planalg/AlternatingDtsp
add_executable(AlternatingDtsp_test planalg/AlternatingDtsp_test.cpp)
target_link_libraries(AlternatingDtsp_test gmock_main OGDF COIN DPP)
add_test(PlanalgTests AlternatingDtsp_test)

# dpp/planalg/NearestNeighborDtsp
add_executable(NearestNeighborDtsp_test planalg/NearestNeighborDtsp_test.cpp)
target_link_libraries(NearestNeighborDtsp_test gmock_main OGDF COIN DPP)
add_test(PlanalgTests NearestNeighborDtsp_test)

# dpp/planner/DubinsVehiclePathPlanner
add_executable(DubinsVehiclePathPlanner_test planner/DubinsVehiclePathPlanner_test.cpp)
target_link_libraries(DubinsVehiclePathPlanner_test gmock_main OGDF COIN DPP)
add_test(PlannerTests DubinsVehiclePathPlanner_test)

# dpp/planner/WaypointSequencePlanner
add_executable(WaypointSequencePlanner_test planner/WaypointSequencePlanner_test.cpp)
target_link_libraries(WaypointSequencePlanner_test gmock_main OGDF COIN DPP)
add_test(PlannerTests WaypointSequencePlanner_test)

