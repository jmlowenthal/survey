#ifndef POSITION2D_H
#define POSITION2D_H

#include <iostream>

struct Position2D {

    float x, y;

    Position2D();

    Position2D(const Position2D& other);

    Position2D(float x, float y);

    Position2D operator+(const Position2D& rhs) const;

    Position2D& operator+=(const Position2D& rhs);

    Position2D operator-(const Position2D& rhs) const;
    
    Position2D& operator-=(const Position2D& rhs);

    Position2D operator*(const float& rhs) const;
    
    Position2D& operator*=(const float& rhs);

    Position2D operator/(const float& rhs) const;
    
    Position2D& operator/=(const float& rhs);

    float magnitudeSq() const;

    float magnitude() const;

    Position2D normalise() const;

    static float distance(const Position2D& a, const Position2D& b);

};

std::ostream& operator<< (std::ostream& os, Position2D const& value);

#endif