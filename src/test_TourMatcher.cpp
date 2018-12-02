#include "catch.hpp"
#include "TourMatcher.h"

TEST_CASE("Duplicate equality") {
    std::vector<Position2D> points{
        Position2D(0, 0),
        Position2D(1, 2),
        Position2D(-1, 3),
        Position2D(0, -2)
    };
    std::vector<Position2D> desired{
        Position2D(0, 0),
        Position2D(1, 2),
        Position2D(-1, 3),
        Position2D(0, -2)
    };
    REQUIRE_THAT(points, TourEqual(desired));
}

TEST_CASE("Circular equality") {
    std::vector<Position2D> points{
        Position2D(-1, 3),
        Position2D(0, -2),
        Position2D(0, 0),
        Position2D(1, 2)
    };
    std::vector<Position2D> desired{
        Position2D(0, 0),
        Position2D(1, 2),
        Position2D(-1, 3),
        Position2D(0, -2)
    };
    REQUIRE_THAT(points, TourEqual(desired));
}