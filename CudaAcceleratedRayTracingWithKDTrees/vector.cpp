#include "vector.h"

ostream& operator<<(ostream& s, const Vector& v)
{
    return s << '[' << v.x << ',' << v.y << ',' << v.z << ']';
}
