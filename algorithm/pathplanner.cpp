#include "pathplanner.h"
#include "communication/udpsender.h"
#include "communication/serialsender.h"

PathPlanner::PathPlanner()
{

}

void PathPlanner::plan()
{
    if(path.size() > 0){
        while (hasArrived(path.front()))
        {
            qDebug() << "has arrived";
            path.pop_front();
        }
        goToPoint(path.front());
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
    if(me_angle > targetAngle)
        rotVel *= -1;
    if(fabs(me_angle - targetAngle) > PARAMS::MATH::PI)
        rotVel *= -1;

    if(fabs(diffAngle) >= PARAMS::ANGLE_THRESHOLD){
        velX = 0;
        velY = 0;
        velW = rotVel;
    }
    else{
        velX = 0;
        velY = 0;
        velW = 0;
    }
}

void PathPlanner::goToPoint(MyPoint target)
{
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    double targetAngle = -atan2(target.y() - me.y, target.x() - me.x);
    double me_angle = MyDataManager::instance()->ourRobot().orientation;

    double diffAngle = fabs(me_angle - targetAngle);
//    qDebug() <<"angle" <<me_angle << MyDataManager::instance()->blueRobots[1].orientation << MyDataManager::instance()->blueRobots[2].orientation <<MyDataManager::instance()->blueRobots[0].orientation;
    diffAngle = diffAngle > PARAMS::MATH::PI ? 2*PARAMS::MATH::PI - diffAngle : diffAngle;
    // 计算旋转速度的绝对值
    double rotVel = diffAngle * PARAMS::FORWARD_ROTATE_COFF; // need improve!!!
    // 计算旋转速度的方向
    if(me_angle > targetAngle)
        rotVel *= -1;
    if(fabs(me_angle - targetAngle) > PARAMS::MATH::PI)
        rotVel *= -1;

    if(fabs(diffAngle) >= PARAMS::FORWARD_ANGLE_THRESHOLD) {
        velX = 0;
        velY = 0;
        velW = rotVel;
    }
    else{
        velX = PARAMS::FORWARD_SPEED;
        velY = 0;
        velW = rotVel;
    }
}

void PathPlanner::stopMoving()
{
    velX = 0;
    velY = 0;
    velW = 0;
}

void PathPlanner::updatePath(std::vector<Node> &nodePath)
{
    path.clear();
    for(int i=0; i<nodePath.size(); i++){
        path.push_back(MyPoint(nodePath[i].x, nodePath[i].y));
    }
}

void PathPlanner::updatePath(std::vector<MyPoint> &pointPath)
{
    path.clear();
    for(int i=0; i<pointPath.size(); i++){
        path.push_back(pointPath[i]);
    }
}

void PathPlanner::clearPath()
{
    path.clear();
}
