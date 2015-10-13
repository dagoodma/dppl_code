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

################################
# Unit Tests
################################

# dpp/basic/Logger
add_executable(Logger_test basic/Logger_test.cpp ../src/dpp/basic/Logger.cpp)
target_link_libraries(Logger_test gmock_main DPP )
add_test(BasicTests Logger_test)

# dpp/basic/Util
add_executable(Util_test basic/Util_test.cpp )
target_link_libraries(Util_test gmock_main OGDF COIN DPP )
add_test(BasicTests Util_test)

# dpp/basic/Path
add_executable(Path_test basic/Path_test.cpp ../src/dpp/basic/Path.cpp)
target_link_libraries(Path_test gmock_main OGDF COIN DPP DUBINSCURVES)
add_test(BasicTests Path_test)

# dpp/planalg/RandomizedDTSP
add_executable(RandomizedDTSP_test planalg/RandomizedDTSP_test.cpp
    ../src/dpp/planalg/RandomizedDTSP.cpp
    ../src/dpp/planalg/Algorithm.cpp
    ../src/dpp/basic/Path.cpp
    ../src/dpp/basic/Logger.cpp)
target_link_libraries(RandomizedDTSP_test gmock_main OGDF COIN DPP DUBINSCURVES)
add_test(PlanalgTests RandomizedDTSP_test)

# Add all tests
#add_test(AllTests BasicTests)

