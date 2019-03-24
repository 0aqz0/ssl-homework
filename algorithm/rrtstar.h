#ifndef RRTSTAR_H
#define RRTSTAR_H
#include <vector>

class Node {
public:
    Node(int x, int y,int parent) {
        this->x = x;
        this->y = y;
        this->parent = parent;
        cost = 0.0;
    }
    int x, y;
    int parent;
    double cost;
};

class RRTStar
{
public:
    RRTStar(){}
    void plan(unsigned char* costs, float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
private:
    int findNearestNode(int x, int y);
    void pathSmoothing();
    std::vector<Node> NodeList;
};

#endif // RRTSTAR_H
