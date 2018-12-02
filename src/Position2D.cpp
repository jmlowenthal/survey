#include "Position2D.h"
#include <math.h>

Position2D::Position2D() {
    x = 0;
    y = 0;
}

Position2D::Position2D(const Position2D& other) {
    x = other.x;
    y = other.y;
}

Position2D::Position2D(float x, float y) {
    this->x = x;
    this->y = y;
}

Position2D Position2D::operator+(const Position2D& rhs) const {
    return Position2D(x + rhs.x, y + rhs.y);
}

Position2D& Position2D::operator+=(const Position2D& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
}

Position2D Position2D::operator-(const Position2D& rhs) const {
    return Position2D(x - rhs.x, y - rhs.y);
}

Position2D& Position2D::operator-=(const Position2D& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
}

Position2D Position2D::operator*(const float& rhs) const {
    return Position2D(x * rhs, y * rhs);
}

Position2D& Position2D::operator*=(const float& rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
}

Position2D Position2D::operator/(const float& rhs) const {
    return Position2D(x / rhs, y / rhs);
}

Position2D& Position2D::operator/=(const float& rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
}

float Position2D::magnitudeSq() const {
    return x * x + y * y;
}

float Position2D::magnitude() const {
    return sqrtf(magnitudeSq());
}

Position2D Position2D::normalise() const {
    return *this / magnitude();
}

float Position2D::distance(const Position2D& a, const Position2D& b) {
    return (a - b).magnitude();
}

std::ostream& operator<< (std::ostream& os, Position2D const& value) {
    os << "(" << value.x << ", " << value.y << ")";
    return os;
}