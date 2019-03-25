#ifndef RRTSTAR_H
#define RRTSTAR_H
#include <vector>
#include "utils/mymath.h"
#include "utils/singleton.hpp"

class RRTStar
{
public:
    RRTStar(){}
    void plan(double start_x, double start_y, double end_x, double end_y);
    std::vector<MyPoint> finalPath;
private:
    int chooseBestParent(int x, int y, double radius, double old_cost);
    int findNearestNode(int x, int y);
//    void findNearNodes(std::vector<Node> &nearNodes);
    void rewire(Node newNode, double radius);
    void pathSmooth();
    bool inNodeList(int x, int y);
    Node randomSample(double epsilon, int end_x, int end_y);
    std::vector<Node> NodeList;
};
typedef Singleton<RRTStar> RRTStarPlanner;

#endif // RRTSTAR_H
