#include "datamanager.h"

DataManager::DataManager()
{
    reset();
}

void DataManager::reset()
{
    for (int i=0; i<PARAMS::ROBOT_NUM; i++){
        validBlueRobots[i] = false;
        validYellowRobots[i] = false;
    }
}

RobotInfo& DataManager::ourRobot()
{
    if (PARAMS::isBlue)
        return blueRobots[PARAMS::our_id];
    else
        return yellowRobots[PARAMS::our_id];
}
