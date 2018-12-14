#include "catch.hpp"
#include "TourMatcher.h"

TEST_CASE("[TourMatcher] Duplicate equality") {
    std::vector<int> points = { 0, 2, 1, 3 };
    std::vector<int> desired = { 0, 2, 1, 3};
    REQUIRE_THAT(points, TourEqual(desired));
}

TEST_CASE("[TourMatcher] Circular equality") {
    std::vector<int> points = { 0, 2, 1, 3 };
    std::vector<int> desired = { 1, 3, 0, 2 };
    REQUIRE_THAT(points, TourEqual(desired));
}

TEST_CASE("[TourMatcher] Negative example") {
    std::vector<int> points = { 0, 1, 2, 3 };
    std::vector<int> desired = { 0, 2, 3, 1 };
    REQUIRE_THAT(points, !TourEqual(desired));
}