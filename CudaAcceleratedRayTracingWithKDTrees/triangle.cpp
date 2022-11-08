#include "triangle.h"

ostream& operator<<(ostream& s, const Triangle& t)
{
    return s << '[' << t.a << ',' << t.b << ',' << t.c << ']';
}

