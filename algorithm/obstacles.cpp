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
            if(MyDataManager::instance()->validBlueRobots[i] && !(i==(PARAMS::our_id-1) && PARAMS::isBlue)){
                RobotInfo& blue = MyDataManager::instance()->blueRobots[i];
                if(pow(blue.x - x, 2) + pow(blue.y - y, 2) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                    return true;
            }
            // yellow robot
            if(MyDataManager::instance()->validYellowRobots[i] && !(i==(PARAMS::our_id-1) && !PARAMS::isBlue)){
                RobotInfo& yellow = MyDataManager::instance()->yellowRobots[i];
                if(pow(yellow.x - x, 2) + pow(yellow.y - y, 2) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                    return true;
            }
        }
        break;
    default:
        break;
    }
    return false;
}

bool Obstacles::hasObstacle(double x1, double y1, double x2, double y2, inflationType type)
{
    double distance = sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2));
    double theta = atan2(y2 - y1, x2 - x1);
    double dx = PARAMS::OBSTACLE::STEP_SIZE*cos(theta);
    double dy = PARAMS::OBSTACLE::STEP_SIZE*sin(theta);
    double x = x1;
    double y = y1;
    switch (type) {
    case CIRCLE:
        for(int i=0; i<distance/PARAMS::OBSTACLE::STEP_SIZE; i++){
            if(hasObstacle(x, y, type))
                return true;
            x += dx;
            y += dy;
        }
        break;
    default:
        break;
    }
    return false;
}
