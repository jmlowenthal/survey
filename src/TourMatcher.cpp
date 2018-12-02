#include "TourMatcher.h"
#include <sstream>

TourMatcher::TourMatcher(std::vector<Position2D>& v) : a(v) {

}

bool TourMatcher::match(std::vector<Position2D> const& b) const {
    if (a.size() != b.size()) {
        return false;
    }
    int i = 0;
    for (int j = 0; j < 2 * a.size(); ++j) {
        if (equal(a[i], b[j % b.size()])) {
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

std::string TourMatcher::describe() const {
    std::ostringstream ss;
    ss << "is equal to { ";
    for (int i = 0; i < a.size(); ++i) {
        ss << a[i];
        if (i < a.size() - 1) {
            ss << ", ";
        }
    }
    ss << " } considering the lists to be circular and wrap-around";
    return ss.str();
}