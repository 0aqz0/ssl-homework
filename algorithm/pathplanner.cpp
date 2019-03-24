#include "pathplanner.h"
#include "communication/udpsender.h"

PathPlanner::PathPlanner()
{
    // test
    path.push_back(MyPoint(300, 0));
    path.push_back(MyPoint(-300, 0));
}

void PathPlanner::plan()
{
    if (path.size() > 0){
        MyPoint current_target = path.front();
        while (hasArrived(current_target))
        {
            path.push_back(path.front());
            path.pop_front();
            current_target = path.front();
        }
        rotateToPoint(current_target);
    }
}

bool PathPlanner::hasArrived(MyPoint target)
{
    return pow(MyDataManager::instance()->ourRobot().x - target.x(), 2)
            + pow(MyDataManager::instance()->ourRobot().y - target.y(), 2)
            <= pow(PARAMS::ACCEPT_RADIUS, 2);
}

void PathPlanner::rotateToPoint(MyPoint target)
{
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    double targetAngle = -atan2(target.y() - me.y, target.x() - me.x);
    double me_angle = MyDataManager::instance()->ourRobot().orientation;

    double diffAngle = fabs(me_angle - targetAngle);
    diffAngle = diffAngle > PARAMS::MATH::PI ? 2*PARAMS::MATH::PI - diffAngle : diffAngle;
    // 计算旋转速度的绝对值
    double rotVel = diffAngle * PARAMS::ROTATE_COFF; // need improve!!!
    // 计算旋转速度的方向

    if (fabs(diffAngle) >= PARAMS::ANGLE_THRESHOLD) {
        CommandSender::instance()->sendToSim(0,0,0,rotVel);
    }
    else
        CommandSender::instance()->sendToSim(0,0,0,0);
}
