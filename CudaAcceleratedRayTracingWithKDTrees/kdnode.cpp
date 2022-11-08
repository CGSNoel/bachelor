#include "kdnode.h"

ostream& operator<<(ostream& s, const KDNode* k) {
    return s << "[" << k->min_x << ", " << k->max_x << ", " << k->min_y << ", " << k->max_y << ", " << k->min_z << ", " << k->max_z << "]";
}
