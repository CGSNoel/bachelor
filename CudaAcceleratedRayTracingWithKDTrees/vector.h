#ifndef VECTOR_H
#define VECTOR_H


#include <math.h>
#include <iostream>
using namespace std;

class Vector {
public:
    explicit Vector(float X = 0, float Y = 0, float Z = 0)
        : x(X), y(Y), z(Z)
    {
    }

    Vector operator+(const Vector& t) const
    {
        return Vector(x + t.x, y + t.y, z + t.z);
    }

    Vector operator+(float f) const
    {
        return Vector(x + f, y + f, z + f);
    }

    friend Vector operator+(float f, const Vector& t)
    {
        return Vector(f + t.x, f + t.y, f + t.z);
    }

    Vector operator-() const
    {
        return Vector(-x, -y, -z);
    }

    Vector operator-(const Vector& t) const
    {
        return Vector(x - t.x, y - t.y, z - t.z);
    }

    Vector operator-(float f) const
    {
        return Vector(x - f, y - f, z - f);
    }

    friend Vector operator-(float f, const Vector& t)
    {
        return Vector(f - t.x, f - t.y, f - t.z);
    }

    Vector operator*(const Vector& t) const
    {
        return Vector(x * t.x, y * t.y, z * t.z);
    }

    Vector operator*(float f) const
    {
        return Vector(x * f, y * f, z * f);
    }

    friend Vector operator*(float f, const Vector& t)
    {
        return Vector(f * t.x, f * t.y, f * t.z);
    }

    Vector operator/(float f) const
    {
        float invf = 1.0 / f;
        return Vector(x * invf, y * invf, z * invf);
    }

    Vector& operator+=(const Vector& t)
    {
        x += t.x;
        y += t.y;
        z += t.z;
        return *this;
    }

    Vector& operator+=(float f)
    {
        x += f;
        y += f;
        z += f;
        return *this;
    }

    Vector& operator-=(const Vector& t)
    {
        x -= t.x;
        y -= t.y;
        z -= t.z;
        return *this;
    }

    Vector& operator-=(float f)
    {
        x -= f;
        y -= f;
        z -= f;
        return *this;
    }

    Vector& operator*=(const float f)
    {
        x *= f;
        y *= f;
        z *= f;
        return *this;
    }

    Vector& operator/=(const float f)
    {
        float invf = 1.0 / f;
        x *= invf;
        y *= invf;
        z *= invf;
        return *this;
    }

    bool operator==(const Vector& t) const
    {
        return std::fabs(x - t.x) < 0.0001 && std::fabs(y - t.y) < 0.0001 && std::fabs(z - t.z) < 0.0001;
    }


    float dot(const Vector& t) const
    {
        return x * t.x + y * t.y + z * t.z;
    }

    Vector cross(const Vector& t) const
    {
        return Vector(y * t.z - z * t.y,
            z * t.x - x * t.z,
            x * t.y - y * t.x);
    }

    float length() const
    {
        return sqrt(length_2());
    }

    float length_2() const
    {
        return x * x + y * y + z * z;
    }

    Vector normalized() const
    {
        return (*this) / length();
    }

    void normalize()
    {
        float l = length();
        float invl = 1 / l;
        x *= invl;
        y *= invl;
        z *= invl;
    }

    friend istream& operator>>(istream& s, Vector& v);
    friend ostream& operator<<(ostream& s, const Vector& v);

    // Functions for when used as a Color:
    void set(float f)
    {
        r = g = b = f;
    }

    void set(float f, float maxValue)
    {
        set(f / maxValue);
    }

    void set(float red, float green, float blue)
    {
        r = red;
        g = green;
        b = blue;
    }

    void set(float r, float g, float b, float maxValue)
    {
        set(r / maxValue, g / maxValue, b / maxValue);
    }

    void clamp(float maxValue = 1.0)
    {
        if (r > maxValue) r = maxValue;
        if (g > maxValue) g = maxValue;
        if (b > maxValue) b = maxValue;
    }

    void c_clamp(float minValue = -1.0, float maxValue = 1.0) {
        if (r > maxValue) r = maxValue;
        if (r < minValue) r = minValue;
        if (g > maxValue) g = maxValue;
        if (g < minValue) g = minValue;
        if (b > maxValue) b = maxValue;
        if (b < minValue) b = minValue;
    }


    Vector reflect(Vector& normal) {
        return *this - 2 * (this->dot(normal) * normal);
    }

    union {
        float data[3];
        struct {
            float x;
            float y;
            float z;
        };
        struct {
            float r;
            float g;
            float b;
        };
    };
};

typedef Vector Color;
typedef Vector Point;


#endif // VECTOR_H
