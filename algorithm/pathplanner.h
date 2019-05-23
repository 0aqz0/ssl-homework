#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include <deque>
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "utils/datamanager.h"
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
    bool moveToNext(MyPoint target);
    void rotateToPoint(MyPoint target);
    void goToPoint(MyPoint target);
    void goToPointTrapezoid(MyPoint target);
    float goToPosition1d( float me2target_dis, float me_vel );
    // return unit is mm, input target unit is cm
    void goToPosition2d( MyPoint target );
    void stopMoving();
    void updatePath(std::vector<Node> &nodePath);
    void updatePath(std::vector<MyPoint> &pointPath);
    void clearPath();
    int pathSize(){ return path.size(); }
    double velX;
    double velY;
    double velW;
    std::deque<MyPoint> path;
};

#endif // PATHPLANNER_H
