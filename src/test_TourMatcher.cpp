#include "catch.hpp"
#include "TourMatcher.h"

TEST_CASE("[TourMatcher] Duplicate equality") {
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

TEST_CASE("[TourMatcher] Circular equality") {
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

TEST_CASE("[TourMatcher] Negative example") {
    std::vector<Position2D> points{
        Position2D(-1, 3),
        Position2D(0, -2),
        Position2D(0, 0),
        Position2D(1, 2)
    };
    std::vector<Position2D> desired{
        Position2D(0, 0),
        Position2D(-1, 3),
        Position2D(1, 2),
        Position2D(0, -2)
    };
    REQUIRE_THAT(points, !TourEqual(desired));
}