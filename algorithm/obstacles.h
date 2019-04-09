#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "utils/singleton.hpp"

enum inflationType{
    CIRCLE = 1,
    CAPSULE = 2,
};

class Obstacles
{
public:
    Obstacles(){}
    bool hasObstacle(double x, double y, inflationType type);
    bool hasObstacle(double x1, double y1, double x2, double y2, inflationType type);
    double distanceofPointandLine(double pointx, double pointy, double startx, double starty, double endx, double endy);
};
typedef Singleton<Obstacles> ObstaclesInfo;

#endif // OBSTACLES_H
