#ifndef TEST_CIRCULARMATCHER_H
#define TEST_CIRCULARMATCHER_H

#include "catch.hpp"
#include <boost/graph/simple_point.hpp>

#ifndef EPSILON
#define EPSILON 1e-9
#endif

class TourMatcher : public Catch::MatcherBase< std::vector<boost::simple_point<float>> > {

private:

    std::vector<boost::simple_point<float>>& a;

    inline bool equal(boost::simple_point<float> const& a, boost::simple_point<float> const& b) const {
        return fabsf(a.x - b.x) < EPSILON && fabsf(a.y - b.y) < EPSILON;
    }

public:

    TourMatcher(std::vector<boost::simple_point<float>>& v);

    virtual bool match(std::vector<boost::simple_point<float>> const& b) const override;
    
    virtual std::string describe() const;

};

inline TourMatcher TourEqual(std::vector<boost::simple_point<float>>& v) {
    return TourMatcher(v);
}

#endif