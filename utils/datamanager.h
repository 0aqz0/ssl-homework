#ifndef DATAMANAGER_H
#define DATAMANAGER_H
#include "utils/singleton.hpp"

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
    RobotInfo blueRobots[16];            // need improve!!!
    RobotInfo yellowRobots[16];
    bool validBlueRobots[16];
    bool validYellowRobots[16];
    void reset();
};
typedef Singleton<DataManager> MyDataManager;

#endif // DATAMANAGER_H
