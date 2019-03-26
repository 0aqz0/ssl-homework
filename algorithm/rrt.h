#ifndef RRT_H
#define RRT_H
#include "utils/mymath.h"
#include "utils/singleton.hpp"
#include <vector>

class RRT
{
public:
    RRT(){}
    void plan(double start_x, double start_y, double end_x, double end_y);
    std::vector<MyPoint> finalPath;
    std::vector<MyPoint> smoothPath;
private:
    int findNearestNode(int x, int y);
    void pathSmooth();
    bool inNodeList(int x, int y);
    Node randomSample(double epsilon, int end_x, int end_y);
    std::vector<Node> NodeList;
};
typedef Singleton<RRT> RRTPlanner;

#endif // RRT_H
