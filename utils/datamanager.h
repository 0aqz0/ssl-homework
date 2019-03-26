#ifndef DATAMANAGER_H
#define DATAMANAGER_H
#include "utils/singleton.hpp"
#include "utils/params.h"
#include "utils/mymath.h"
#include <deque>

struct BallInfo
{
    float vel_x;
    float vel_y;
    float x;
    float y;
    bool valid;
    BallInfo() : vel_x(9999), vel_y(9999), x(9999), y(9999), valid(false){}
};

struct RobotInfo
{
    int robot_id;
    float x;
    float y;
    float orientation;
    float vel_x;
    float vel_y;
    float rotate_vel;
    RobotInfo() : robot_id(9999), x(9999), y(9999), orientation(9999),
    vel_x(9999), vel_y(9999), rotate_vel(9999){}
};

class DataManager
{
public:
    DataManager();
    BallInfo ball;
    RobotInfo blueRobots[PARAMS::ROBOT_NUM];
    RobotInfo yellowRobots[PARAMS::ROBOT_NUM];
    bool validBlueRobots[PARAMS::ROBOT_NUM];
    bool validYellowRobots[PARAMS::ROBOT_NUM];
    std::deque<MyPoint> goals;
    RobotInfo& ourRobot();
    void setGoals(std::deque<MyPoint> new_goals);
    void reset();
};
typedef Singleton<DataManager> MyDataManager;

#endif // DATAMANAGER_H
