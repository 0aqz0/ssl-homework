#include "pathplanner.h"
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "obstacles.h"

void PathPlanner::plan()
{
    while (path.size() > 0 && hasArrived(path.front()))
    {
        if(PARAMS::DEBUG::pathPlannerDebug)
            qDebug() << "has arrived";
        path.pop_front();
    }
    if(PARAMS::DEBUG::pathPlannerDebug)
        qDebug() << "path size:" << path.size();
    if(path.size() > 0)
        goToPoint(path.front());
}

bool PathPlanner::hasArrived(MyPoint target)
{
    if(PARAMS::DEBUG::pathPlannerDebug)
        qDebug() << "has arrived";

    return pow(MyDataManager::instance()->ourRobot().x - target.x(), 2)
            + pow(MyDataManager::instance()->ourRobot().y - target.y(), 2)
            <= pow(PARAMS::ACCEPT_RADIUS, 2);
}

//// 判断机器人是否需要走下一个点
//bool PathPlanner::moveToNext(MyPoint target)
//{
//    bool next = false;
//    if(hasArrived(target))
//        next = true;
//    // 解决机器人走过头的问题
//    if(path.size() > 1){
//        MyPoint nextTarget = path[1];
//        RobotInfo& me = MyDataManager::instance()->ourRobot();
//        double distToTarget = sqrt(pow(me.x - target.x(),2) + pow(me.y - target.y(), 2))
//                + sqrt(pow(target.x() - nextTarget.x(),2) + pow(target.y() - nextTarget.y(),2));
//        double distToNextTarget = sqrt(pow(me.x - nextTarget.x(),2) + pow(me.x - nextTarget.x(),2));
//        if(distToTarget > distToNextTarget && !ObstaclesInfo::instance()->hasObstacle(me.x, me.y, nextTarget.x(), nextTarget.y(), CIRCLE))
//            next = true;
////        qDebug() << "jump!!!";
//    }
//    return next;
//}

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
    if(PARAMS::DEBUG::pathPlannerDebug)
        qDebug() << "go to point";
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

void PathPlanner::updatePath(std::vector<MyPoint> &pointPath)
{
    if(PARAMS::DEBUG::pathPlannerDebug)
        qDebug() << "update path begin";
    path.clear();
    for(int i=0; i<pointPath.size(); i++)
        path.push_back(pointPath[i]);
    if(PARAMS::DEBUG::pathPlannerDebug)
        qDebug() << "update path finish";
}

void PathPlanner::clearPath()
{
    path.clear();
}
