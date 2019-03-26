#ifndef OBSTACLES_H
#define OBSTACLES_H
#include "utils/singleton.hpp"

enum inflationType{
    CIRCLE = 1,
};

class Obstacles
{
public:
    Obstacles();
    bool hasObstacle(double x, double y, inflationType type);
};
typedef Singleton<Obstacles> ObstaclesInfo;

#endif // OBSTACLES_H
