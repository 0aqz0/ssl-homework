#include "obstacles.h"
#include "utils/params.h"
#include "utils/datamanager.h"

bool Obstacles::hasObstacle(double x, double y, inflationType type)
{
    switch (type) {
    case CIRCLE:
        for(int i=0; i<PARAMS::ROBOT_NUM; i++){
            // blue robot
            if(MyDataManager::instance()->validBlueRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id) : i==(PARAMS::our_id-1)) && PARAMS::isBlue)){
                RobotInfo& blue = MyDataManager::instance()->blueRobots[i];
                if(pow(blue.x - x, 2) + pow(blue.y - y, 2) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                    return true;
            }
            // yellow robot
            if(MyDataManager::instance()->validYellowRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id) : i==(PARAMS::our_id-1)) && !PARAMS::isBlue)){
                RobotInfo& yellow = MyDataManager::instance()->yellowRobots[i];
                if(pow(yellow.x - x, 2) + pow(yellow.y - y, 2) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                    return true;
            }
        }
        break;
    case CAPSULE:
        for(int i=0; i<PARAMS::ROBOT_NUM; i++){
            // blue robot
            if(MyDataManager::instance()->validBlueRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id) : i==(PARAMS::our_id-1)) && PARAMS::isBlue)){
                RobotInfo& blue = MyDataManager::instance()->blueRobots[i];
                //尤其注意，这里是路子豪写的，是按照正常情况下的X-Y坐标系来做的，对于仿真和实际的环境有点痴迷，详情请询问路子豪
                double globalvx = blue.vel_x * cos(blue.orientation) + blue.vel_y * sin(blue.orientation);
                double globalvy = blue.vel_x * sin(blue.orientation) - blue.vel_y * cos(blue.orientation);
                double endx = blue.x + PARAMS::OBSTACLE::REFRESH_TIME * globalvx;
                double endy = blue.y + PARAMS::OBSTACLE::REFRESH_TIME * globalvy;
                if(distanceofPointandLine(x, y, blue.x, blue.y, endx, endy) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                    return true;
            }
            // yellow robot
            if(MyDataManager::instance()->validYellowRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id) : i==(PARAMS::our_id-1)) && !PARAMS::isBlue)){
                RobotInfo& yellow = MyDataManager::instance()->yellowRobots[i];
                //尤其注意，这里是路子豪写的，是按照正常情况下的X-Y坐标系来做的，对于仿真和实际的环境有点痴迷，详情请询问路子豪
                double globalvx = yellow.vel_x * cos(yellow.orientation) + yellow.vel_y * sin(yellow.orientation);
                double globalvy = yellow.vel_x * sin(yellow.orientation) - yellow.vel_y * cos(yellow.orientation);
                double endx = yellow.x + PARAMS::OBSTACLE::REFRESH_TIME * globalvx;
                double endy = yellow.y + PARAMS::OBSTACLE::REFRESH_TIME * globalvy;
                if(distanceofPointandLine(x, y, yellow.x, yellow.y, endx, endy) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
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

double Obstacles::distanceofPointandLine(double pointx, double pointy, double startx, double starty, double endx, double endy)
{
    if (startx == endx && starty == endy)
        return sqrt(pow(endx-pointx,2)+pow(endy-pointy,2));
    double r = (pointx-startx)*(endx-startx) + (pointy-starty)*(endy-starty);
    if(r <= 0) return sqrt(pow(startx-pointx,2)+pow(starty-pointy,2));
    else if(r >= 1) return sqrt(pow(endx-pointx,2)+pow(endy-pointy,2));
    else{
        //通过斜率求解
        double k =  (endy-starty) / (endx-startx);
        return fabs(starty-pointy - k*(startx-pointx))/sqrt(k*k+1);
    }
}
