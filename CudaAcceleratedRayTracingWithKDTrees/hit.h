#ifndef HIT_H
#define HIT_H

#include "vector.h"
#include <limits>


class Triangle;
class Hit
{
public:
    float t;
    Vector N;
    Triangle* hitobject;
    bool no_hit;

    Hit();

    Hit(float t, const Vector& normal, Triangle* hitobject = nullptr, bool nohit = false)
        : t(t), N(normal), hitobject(hitobject), no_hit(nohit)
    { }

    static const Hit NO_HIT() { static Hit no_hit(std::numeric_limits<float>::quiet_NaN(), Vector(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()), nullptr, true); return no_hit; }

};

#endif // HIT_H
