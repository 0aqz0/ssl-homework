#ifndef RRTSTAR_H
#define RRTSTAR_H
#include <vector>
#include "utils/mymath.h"
#include "utils/singleton.hpp"

class RRTStar
{
public:
    RRTStar(){}
    void plan(unsigned char* costs, float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
private:
    int findNearestNode(int x, int y);
    void pathSmooth();
    std::vector<Node> NodeList;
};
typedef Singleton<RRTStar> RRTStarPlanner;

#endif // RRTSTAR_H
