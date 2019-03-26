#include "obstacles.h"
#include "utils/params.h"
#include "utils/datamanager.h"

Obstacles::Obstacles()
{

}

bool Obstacles::hasObstacle(double x, double y, inflationType type)
{
    switch (type) {
    case CIRCLE:
        for(int i=0; i<PARAMS::ROBOT_NUM; i++){
            // blue robot
//            if(MyDataManager::instance()->validBlueRobots[i]){
//                RobotInfo& blue = MyDataManager::instance()->blueRobots[i];
//                if(pow(blue.x - x, 2) + pow(blue.y - y, 2) <= pow(PARAMS::OBSTACLE::INFLATION_RADIUS, 2))
//                    return true;
//            }
            // yellow robot
//            if(MyDataManager::instance()->validYellowRobots[i]){
//                RobotInfo& yellow = MyDataManager::instance()->yellowRobots[i];
//                if(pow(yellow.x - x, 2) + pow(yellow.y - y, 2) <= pow(PARAMS::OBSTACLE::INFLATION_RADIUS, 2))
//                    return true;
//            }
        }
        break;
    default:
        break;
    }
    return false;
}

bool Obstacles::hasObstacle(double x1, double y1, double x2, double y2, inflationType type)
{
    return false;
}
