#ifndef RRT_H
#define RRT_H
#include "utils/mymath.h"
#include "utils/singleton.hpp"
#include <vector>

class RRT
{
public:
    RRT(){}
    void plan(unsigned char* costs, float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
private:
    int findNearestNode(int x, int y);
    void pathSmooth();
    std::vector<Node> NodeList;
};
typedef Singleton<RRT> RRTPlanner;

#endif // RRT_H
