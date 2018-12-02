#ifndef TEST_CIRCULARMATCHER_H
#define TEST_CIRCULARMATCHER_H

#include "catch.hpp"
#include "Position2D.h"

#ifndef EPSILON
#define EPSILON 1e-9
#endif

class TourMatcher : public Catch::MatcherBase< std::vector<Position2D> > {

private:

    std::vector<Position2D>& a;

    inline bool equal(Position2D const& a, Position2D const& b) const {
        return fabsf(a.x - b.x) < EPSILON && fabsf(a.y - b.y) < EPSILON;
    }

public:

    TourMatcher(std::vector<Position2D>& v);

    virtual bool match(std::vector<Position2D> const& b) const override;
    
    virtual std::string describe() const;

};

inline TourMatcher TourEqual(std::vector<Position2D>& v) {
    return TourMatcher(v);
}

#endif