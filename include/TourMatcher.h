#ifndef TEST_CIRCULARMATCHER_H
#define TEST_CIRCULARMATCHER_H

#include "catch.hpp"
#include <sstream>

template<typename T>
class TourMatcher : public Catch::MatcherBase< std::vector<T> > {

private:

    const std::vector<T>& a;

public:

    TourMatcher(const std::vector<T>& v) : a(v) {}

    virtual bool match(const std::vector<T>& b) const override {
        if (a.size() != b.size()) {
            return false;
        }
        int i = 0;
        for (int j = 0; j < 2 * a.size(); ++j) {
            if (a[i] == b[j % b.size()]) {
                ++i;
                if (i >= a.size()) {
                    return true;
                }
            }
            else {
                i = 0;
            }
        }
        for (int j = 2 * a.size() - 1; j >= 0; --j) {
            if (a[i] == b[j % b.size()]) {
                ++i;
                if (i >= a.size()) {
                    return true;
                }
            }
            else {
                i = 0;
            }
        }
        return false;
    }
    
    virtual std::string describe() const {
        std::ostringstream ss;
        ss << "is equal to { ";
        for (int i = 0; i < a.size(); ++i) {
            ss << a[i];
            if (i < a.size() - 1) {
                ss << ", ";
            }
        }
        ss << " } considering the lists to be circular, wrap-around and reversable";
        return ss.str();
    }

};

template<typename T>
inline TourMatcher<T> TourEqual(std::vector<T>& v) {
    return TourMatcher<T>(v);
}

#endif