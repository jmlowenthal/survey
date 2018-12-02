#[[
    Run `tests [#filename] -#` to run the tests from a particular file, for
    example `tests [#test_Position2D] -#`.
]]

cmake_minimum_required (VERSION 3.0)

project (GlobalPlanner)
set(CMAKE_BINRY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
include_directories(include/ lib/catch2/include/)

set(
    CORE_SRC
    src/LinKernighanTspSolver.cpp
    src/Position2D.cpp
    src/RandomTspSolver.cpp
)

add_executable(
    planner
    ${CORE_SRC}
    src/main.cpp
)

add_executable(
    tests
    ${CORE_SRC}
    src/TourMatcher.cpp
    src/test_main.cpp
    src/test_LinKernighanTspSolver.cpp
    src/test_TourMatcher.cpp
    src/test_Position2D.cpp
)

set(CMAKE_MODULE_PATH lib/catch2/cmake/)
include(CTest)
include(Catch)
catch_discover_tests(tests)