#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include <vector>
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "utils/datamanager.h"

class Point
{
public:
    Point(int x=0, int y=0) : _x(x), _y(y) {}
    int x() const { return _x; }
    int y() const { return _y; }
private:
    int _x;
    int _y;
};

class PathPlanner
{
public:
    PathPlanner();
    void plan();
    bool hasArrived(Point target);
private:
    std::vector<Point> path;
};

#endif // PATHPLANNER_H
