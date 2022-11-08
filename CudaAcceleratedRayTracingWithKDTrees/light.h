#ifndef LIGHT_H
#define LIGHT_H

#include "vector.h"

class Light
{
public:
    Light(Point pos, Color c) : position(pos), color(c)
    { }

    Point position;
    Color color;
};

#endif // LIGHT_H
