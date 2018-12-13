#[[
    Run `cmake -DCMAKE_BUILD_TYPE=[Release|Debug]` to create Makefile, then run
    `make`. The `test` target (run `make test`) will execute the test but
    running the `tests` executable will give more verbose output.

    Run `tests [#filename] -#` to run the tests from a particular file, for
    example `tests [#test_Position2D] -#`.
]]

cmake_minimum_required (VERSION 3.0)

project (GlobalPlanner)
set(CMAKE_BINRY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
include_directories(include/ lib/catch2/include/)

# Boost libraries
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_ENABLE_ASSERT_DEBUG_HANDLER)

set(
    CORE_SRC
    src/Position2D.cpp
    src/AntColonySystem.cpp
    src/RandomTspSolver.cpp
    src/NearestNeighbourTspSolver.cpp
    src/assert.cpp
)

add_executable(
    planner
    ${CORE_SRC}
    src/main.cpp
)
target_link_libraries(planner ${Boost_LIBRARIES})

add_executable(
    tests
    ${CORE_SRC}
    src/TourMatcher.cpp
    src/test_main.cpp
    src/test_AntColonySystem.cpp
    src/test_TourMatcher.cpp
    src/test_Position2D.cpp
    src/test_SymmetricPairwiseMatrix.cpp
)
target_link_libraries(tests ${Boost_LIBRARIES})

set(CMAKE_MODULE_PATH lib/catch2/cmake/)
include(CTest)
include(Catch)
catch_discover_tests(tests)