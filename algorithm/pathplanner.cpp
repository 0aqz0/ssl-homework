#include "pathplanner.h"
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "obstacles.h"
#include <iostream>
#include <iomanip>

namespace {
const float DIRECTION_ACCURACY = 2 / 180 * PARAMS::MATH::PI;
const float DELTA_TIME = 0.03; // unit is second
const float ACC_BUFFER = 1.2;
const float STOP_BUFFER_UP = 400;
const float STOP_BUFFER_DOWN = 0;
const float ROTATE = 5;
}

void PathPlanner::plan()
{
    while (path.size() > 0 && moveToNext(path.front()))
    {
        if(PARAMS::DEBUG::pathPlannerDebug)
            qDebug() << "has arrived";
        path.pop_front();
    }
    if(PARAMS::DEBUG::pathPlannerDebug)
        qDebug() << "path size:" << path.size();
    if(path.size() > 0)
//        goToPoint(path.front());
//        goToPointTrapezoid(path.front());
        goToPosition2d(path.front());
}

bool PathPlanner::hasArrived(MyPoint target)
{
    if(PARAMS::DEBUG::pathPlannerDebug)
        qDebug() << "has arrived";

    return pow(MyDataManager::instance()->ourRobot().x - target.x(), 2)
            + pow(MyDataManager::instance()->ourRobot().y - target.y(), 2)
            <= pow(PARAMS::ACCEPT_RADIUS, 2);
}

// 判断机器人是否需要走下一个点
bool PathPlanner::moveToNext(MyPoint target)
{
    bool next = false;
    if(hasArrived(target))
        next = true;
    // 解决机器人走过头的问题
    if(path.size() > 1){
        MyPoint nextTarget = path[1];
        RobotInfo& me = MyDataManager::instance()->ourRobot();
        double distToTarget = sqrt(pow(me.x - target.x(),2) + pow(me.y - target.y(), 2))
                + sqrt(pow(target.x() - nextTarget.x(),2) + pow(target.y() - nextTarget.y(),2));
        double distToNextTarget = sqrt(pow(me.x - nextTarget.x(),2) + pow(me.x - nextTarget.x(),2));
        if(distToTarget > distToNextTarget && !ObstaclesInfo::instance()->hasObstacle(me.x, me.y, nextTarget.x(), nextTarget.y(), CIRCLE))
            next = true;
    }
    return next;
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

void PathPlanner::goToPointTrapezoid(MyPoint target)
{
    //尤其注意，本函数得出的velx和vely和实际场地是相同的，不需要乘-1，可以乘倍数
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    double targetAngle = atan2(target.y() - me.y, target.x() - me.x);
    double globalvx = PARAMS::FORWARD_SPEED * cos(targetAngle);
    double globalvy = PARAMS::FORWARD_SPEED * sin(targetAngle);
    double localvx = globalvx * cos(me.orientation) - globalvy * sin(me.orientation);
    double localvy = globalvx * sin(me.orientation) + globalvy * cos(me.orientation);


//    float dis = sqrt(pow(target.x()-me.x,2)+pow(target.y()-me.y,2));
//    float vel_now = sqrt(pow(me.vel_x,2)+pow(me.vel_y,2));
//    float dismax = vel_now/PARAMS::multiple_realToCommand*15;
//    qDebug() << "dis" << dis << "dismax" << dismax;
//    if(dis < dismax && false)
//    {
//        //it's good in reality, but not in simulink.
//        velX = fmax(localvx,2.0);
//        velY = fmax(localvy,2.0);
//    }
//    else{
        velX = localvx;
        velY = localvy;
//    }
//    velW = 0;
//    qDebug() << "now goToPoint-------------ing";
}

void PathPlanner::goToPosition2d( MyPoint target ){
    if ( PARAMS::DEBUG::kgoToPosition2d ){
        std::cout << "\n=================  [pathplanner.cpp] debug  =========="
                        "========" << std::endl;
    }
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    MyPoint me_pos( me.x * 10, me.y * 10 );
    target.Setx( target.x() * 10 );
    target.Sety( target.y() * 10 );
    MyVector me2target = target - me_pos;
    MyVector me_vel( me.vel_x, me.vel_y );
    MyVector next_step;

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

    velW = rotVel;

    if ( me_vel.mod() < 300 ){
        // our robot stop
        next_step =  me_vel + me2target.Unitization() * PARAMS::FORWARD_ACC *
                1000 * DELTA_TIME;
    }

//    else if ( angular_bias < DIRECTION_ACCURACY ){
    else {
        // the direction of our robot's velocity is toward target
        float next_step_mod = 0;
        next_step_mod = goToPosition1d( me2target.mod(), me_vel.mod() );
        next_step = me2target.Unitization() * next_step_mod;
    }

//    else if ( angular_bias >= DIRECTION_ACCURACY &&
//              angular_bias < PARAMS::MATH::PI ) {
//        // The Angle between our vehicle speed and the target direction is
//        // greater than DIRECTION_ACCURACY and is less than 90 degree

//    }

//    else {
//        // The Angle between our vehicle speed and the target direction is
//        // greater than 90 degree

//    }
    velX = next_step.x() * cos(me.orientation) - next_step.y() * sin(me.orientation);
    velY = next_step.x() * sin(me.orientation) + next_step.y() * cos(me.orientation);
//    velX = next_step.x() * cos(me.orientation) + next_step.y() * sin(me.orientation);
//    velY = next_step.x() * sin(me.orientation) - next_step.y() * cos(me.orientation);
    velX = velX / 1000;
    velY = velY / 1000;
    if ( PARAMS::DEBUG::kgoToPosition2d ){
        std::cout /*<< "next_step : " << std::right << std::setw(12) << next_step.x()
                  << ", next_step: " << std::right << std::setw(12) << next_step.y()*/
                  << "\nme_vel_x : " << std::right << std::setw(12) << me_vel.x()
                  << ", me_vel_y : " << std::right << std::setw(12) << me_vel.y()
                  << std::endl;
        std::cout << "\n=========================================================="
                        "=====" << std::endl;
    }
}

float PathPlanner::goToPosition1d( float me2target_dis, float me_vel){
    float v_max = PARAMS::FORWARD_SPEED * 1000;
    float a_max = PARAMS::FORWARD_ACC * 1000;
    float stop_dis = me_vel * me_vel / 2 / a_max;

    if ( PARAMS::DEBUG::kgoToPosition2d ){
//        std::cout << "stop_dis : " << std::right << std::setw(12) << stop_dis / ACC_BUFFER
//                  << ", me2target_dis: " << std::right << std::setw(12) << me2target_dis
//                  << "\nBUFFER: " << ACC_BUFFER << std::endl;
    }
    float STOP_BUFFER;
    if ( me_vel > 1500 ){
        STOP_BUFFER = STOP_BUFFER_UP;
    }
    else {
//        std::cout << "[pathplanner.cpp] test" << std::endl;
        STOP_BUFFER = STOP_BUFFER_DOWN;
    }
    if ( me2target_dis <= stop_dis + STOP_BUFFER ){
        return me_vel - a_max * DELTA_TIME * ACC_BUFFER;
    }
    else if ( (v_max - me_vel) > a_max * DELTA_TIME ) {
        return me_vel + a_max * DELTA_TIME;
    }
    else {
        return v_max;
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
    //不要起点
    for(int i=1; i<pointPath.size(); i++)
        path.push_back(pointPath[i]);
    if(PARAMS::DEBUG::pathPlannerDebug)
        qDebug() << "update path finish";
}

void PathPlanner::clearPath()
{
    path.clear();
}
