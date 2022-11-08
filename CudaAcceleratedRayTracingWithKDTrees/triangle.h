#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object.h"
#include "vector.h"
#include "hit.h"

class Triangle : public Object
{
public:
    Triangle(Point A, Point B, Point C) : a(A), b(B), c(C), color(Color(1.0, 0, 0)), matDiff(1.0f), matSpec(0.0f), refrIdx(0.0f)
    {}
    Triangle(Point A, Point B, Point C, Color COLOR) : a(A), b(B), c(C), color(COLOR), matDiff(1.0f), matSpec(0.0f), refrIdx(0.0f)
    {}
    Triangle(Point A, Point B, Point C, Color COLOR, float MATDIFF, float MATSPEC, float REFRIDX) : a(A), b(B), c(C), color(COLOR), matDiff(MATDIFF), matSpec(MATSPEC), refrIdx(REFRIDX)
    {}

    Point a;
    Point b;
    Point c;
    Color color;
    float matDiff;
    float matSpec;
    float refrIdx;

    float min_x() {
        return min(min(a.x, b.x), c.x);
    }

    float min_y() {
        return min(min(a.y, b.y), c.y);
    }

    float min_z() {
        return min(min(a.z, b.z), c.z);
    }

    float max_x() {
        return max(max(a.x, b.x), c.x);
    }

    float max_y() {
        return max(max(a.y, b.y), c.y);
    }

    float max_z() {
        return max(max(a.z, b.z), c.z);
    }


    Hit intersect(Ray ray) {


        Vector E0 = b - a;
        //Vector E1 = pointc-pointb;
        Vector E2 = c - a;
        Vector N = E0.cross(E2);
        float det = E0.dot(ray.D.cross(E2));



        if (std::fabs(det) < 1e-6) return Hit::NO_HIT();
        float invdet = 1 / det;
        Vector tvec = ray.O - a;
        Vector qvec = tvec.cross(E0);


        float u = tvec.dot(ray.D.cross(E2)) * invdet;
        if (u < 0 || u>1) return Hit::NO_HIT();
        float v = ray.D.dot(qvec) * invdet;
        if (v < 0 || u + v>1) return Hit::NO_HIT();

        float t = E2.dot(qvec) * invdet;
        if (t < 0) return Hit::NO_HIT();
        return Hit(t, N.normalized(), this);

    }
};

#endif // TRIANGLE_H
