// Grid Coordinate

#ifndef COORD_H_
#define COORD_H_

#include <math.h>

struct Coord {
    float _x;
    float _y;

    Coord() : _x(0.f), _y(0.f) {}
    Coord(float x, float y) : _x(x), _y(y) {}

    static constexpr float epsilon = 0.0000001f;

    bool operator==(const Coord& other) {
        return floatsEqual(_x, other._x) && floatsEqual(_y, other._y);
    }

private:
    static bool floatsEqual(float a, float b) {
        return abs(a-b) < epsilon;
    }
};


#endif  // COORD_H_
