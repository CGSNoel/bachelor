#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray
{
public:
    Point O;
    Vector D;

    Ray(const Point& origin, const Vector& direction)
        : O(origin), D(direction)
    { }

    Point at(float t) const
    {
        return O + t * D;
    }
};

#endif // RAY_H
