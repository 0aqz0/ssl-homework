#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include <deque>
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "utils/datamanager.h"
#include "utils/singleton.hpp"
#include "utils/mymath.h"

/**************************************************************/
/*                         速度规划                            */
/*          Description: 作为将路径点转换为具体下发速度的API       */
/**************************************************************/

class PathPlanner
{
public:
    PathPlanner();
    void plan();
    bool hasArrived(MyPoint target);
    void rotateToPoint(MyPoint target);
    void goToPoint(MyPoint target);
    void stopMoving();
    void updatePath(std::vector<Node> &nodePath);
    void updatePath(std::vector<MyPoint> &pointPath);
private:
    std::deque<MyPoint> path;
};
typedef Singleton<PathPlanner> LocalPlanner;

#endif // PATHPLANNER_H
