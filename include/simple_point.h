#ifndef EQUALITY_H
#define EQUALITY_H

#include <boost/graph/simple_point.hpp>
#include <ostream>

template<typename T>
inline bool operator==(const boost::simple_point<T>& a, const boost::simple_point<T>& b) {
    return a.x == b.x && a.y == b.y;
}

template<typename T>
inline std::ostream& operator<<(std::ostream& stream, const boost::simple_point<T>& a) {
    return stream << "(" << a.x << ", " << a.y << ")";
}

#endif