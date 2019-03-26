#ifndef ARTIFICAL_POTENTIAL_H
#define ARTIFICAL_POTENTIAL_H

#include "utils/singleton.hpp"
#include "utils/mymath.h"

class ArtificalPotential{
public:
//    void plan( MyPoint target, MyVector v_target );
    void plan( MyPoint target );
};

typedef Singleton<ArtificalPotential> ApPlanner;

#endif // ARTIFICAL_POTENTIAL_H
