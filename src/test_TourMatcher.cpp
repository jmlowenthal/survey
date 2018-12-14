#include "catch.hpp"
#include "TourMatcher.h"
#include <boost/graph/simple_point.hpp>

TEST_CASE("[TourMatcher] Duplicate equality") {
    std::vector<boost::simple_point<float>> points{
        { 0, 0 },
        { 1, 2 },
        { -1, 3 },
        { 0, -2 }
    };
    std::vector<boost::simple_point<float>> desired{
        { 0, 0 },
        { 1, 2 },
        { -1, 3 },
        { 0, -2 }
    };
    REQUIRE_THAT(points, TourEqual(desired));
}

TEST_CASE("[TourMatcher] Circular equality") {
    std::vector<boost::simple_point<float>> points{
        { -1, 3 },
        { 0, -2 },
        { 0, 0 },
        { 1, 2 }
    };
    std::vector<boost::simple_point<float>> desired{
        { 0, 0 },
        { 1, 2 },
        { -1, 3 },
        { 0, -2 }
    };
    REQUIRE_THAT(points, TourEqual(desired));
}

TEST_CASE("[TourMatcher] Negative example") {
    std::vector<boost::simple_point<float>> points{
        { -1, 3 },
        { 0, -2 },
        { 0, 0 },
        { 1, 2 }
    };
    std::vector<boost::simple_point<float>> desired{
        { 0, 0 },
        { -1, 3 },
        { 1, 2 },
        { 0, -2 }
    };
    REQUIRE_THAT(points, !TourEqual(desired));
}