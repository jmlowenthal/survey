cmake_minimum_required (VERSION 3.0)

project (GlobalPlanner)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_CXX_STANDARD 17)

include_directories(include/ lib/catch2/include/)

# Boost libraries
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost REQUIRED COMPONENTS system graph)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_ENABLE_ASSERT_DEBUG_HANDLER)

set(
    CORE_SRC
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
    src/test_main.cpp
    src/test_TourMatcher.cpp
    src/test_acs_metric_tsp_approx.cpp
    src/test_nearest_neighbour_metric_tsp_approx.cpp
    # src/test_acs_metric_sop_approx.cpp
    src/test_ada_star_search.cpp
    src/test_functional_graph.cpp
)
target_link_libraries(tests ${Boost_LIBRARIES})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/lib/catch2/cmake/)
include(CTest)
include(Catch)
catch_discover_tests(tests)