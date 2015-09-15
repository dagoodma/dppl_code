# CMakeLists
#cmake_minimum_required(VERSION 3.2)
#project (test CXX)
#enable_testing()

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

# Build tests
#add_executable(MyFirstTest myFirstTest.cpp)
#target_link_libraries(MyFirstTest gmock_main)
#add_test(AllTests MyFirstTest)
add_executable(DubinsUnitTest DubinsUnitTest.cpp ../src/Dubins.cpp)
target_link_libraries(DubinsUnitTest gmock_main OGDF COIN)
add_test(AllTests DubinsUnitTest)
