#ifndef ARTIFICAL_POTENTIAL_H
#define ARTIFICAL_POTENTIAL_H

#include "utils/singleton.hpp"
#include "utils/mymath.h"

class ArtificalPotential{
public:
//    void plan( MyPoint target, MyVector v_target );
    bool plan( MyPoint target );
    float v_x = 0;
    float v_y = 0;
    float v_w = 0;
};

typedef Singleton<ArtificalPotential> ApPlanner;

#endif // ARTIFICAL_POTENTIAL_H
