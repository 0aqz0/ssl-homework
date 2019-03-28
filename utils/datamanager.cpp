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
        return blueRobots[PARAMS::IS_SIMULATION ? PARAMS::our_id : PARAMS::our_id-1];
    else
        return yellowRobots[PARAMS::IS_SIMULATION ? PARAMS::our_id : PARAMS::our_id-1];
}

void DataManager::setGoals(std::deque<MyPoint> new_goals)
{
    goals = new_goals;
}
