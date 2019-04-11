#include "obstacles.h"
#include "utils/params.h"
#include "utils/datamanager.h"

bool Obstacles::hasObstacle(double x, double y, int type)
{
    switch (type) {
    case 1:
        for(int i=0; i<PARAMS::ROBOT_NUM; i++){
            // blue robot
            if(MyDataManager::instance()->validBlueRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id-1) : i==(PARAMS::our_id-1)) && PARAMS::isBlue)){
                RobotInfo& blue = MyDataManager::instance()->blueRobots[i];
                if(pow(blue.x - x, 2) + pow(blue.y - y, 2) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                    return true;
            }
            // yellow robot
            if(MyDataManager::instance()->validYellowRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id-1) : i==(PARAMS::our_id-1)) && !PARAMS::isBlue)){
                RobotInfo& yellow = MyDataManager::instance()->yellowRobots[i];
                if(pow(yellow.x - x, 2) + pow(yellow.y - y, 2) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                    return true;
            }
        }
        break;
    case 2:
        for(int i=0; i<PARAMS::ROBOT_NUM; i++){
            // blue robot
            //qDebug() << "i" << i << "NUM" << PARAMS::ROBOT_NUM;
            if(MyDataManager::instance()->validBlueRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id-1) : i==(PARAMS::our_id-1)) && PARAMS::isBlue))
            {
                RobotInfo& blue = MyDataManager::instance()->blueRobots[i];
                //尤其注意，这里是路子豪写的，是按照正常情况下的X-Y坐标系来做的，对于仿真和实际的环境有点痴迷，详情请询问路子豪
                double globalvx = blue.vel_x;
                double globalvy = -blue.vel_y;
                if(pow(globalvx,2)+pow(globalvy,2) <= 10000)//这是静态的东西
                {
//                    qDebug() << i;
                    if(pow(blue.x - x, 2) + pow(blue.y - y, 2) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                        return true;
                }
                else
                {
                    double endx = blue.x + PARAMS::OBSTACLE::REFRESH_TIME * globalvx;
                    double endy = blue.y + PARAMS::OBSTACLE::REFRESH_TIME * globalvy;
                    //qDebug() << "x" << x << "y" << y << "blue.x" << blue.x << "blue.y" << blue.y << "endx" << endx << "endy" << endy;
                    //qDebug() << "the dis" <<distanceofPointandLine(x, y, blue.x, blue.y, endx, endy);
                    //qDebug() << "INFLATION_RADIUS" << PARAMS::OBSTACLE::INFLATION_RADIUS;
                    if(distanceofPointandLine(x, y, blue.x, blue.y, endx, endy) <= PARAMS::OBSTACLE::INFLATION_RADIUS)
                    {
                        //qDebug() << "true";
                        return true;
                    }
                    else
                    {
                        //qDebug() << "false";
                    }
                }
            }
            // yellow robot
            if(MyDataManager::instance()->validYellowRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id-1) : i==(PARAMS::our_id-1)) && !PARAMS::isBlue))
            {
                RobotInfo& yellow = MyDataManager::instance()->yellowRobots[i];
                //尤其注意，这里是路子豪写的，是按照正常情况下的X-Y坐标系来做的，对于仿真和实际的环境有点痴迷，详情请询问路子豪
                double globalvx = yellow.vel_x;
                double globalvy = -yellow.vel_y;
                if(pow(globalvx,2)+pow(globalvy,2) <= 10000)
                {
                    if(pow(yellow.x - x, 2) + pow(yellow.y - y, 2) <= PARAMS::OBSTACLE::INFLATION_RADIUS_SQUARE)
                        return true;
                }
                else
                {
                    double endx = yellow.x + PARAMS::OBSTACLE::REFRESH_TIME * globalvx;
                    double endy = yellow.y + PARAMS::OBSTACLE::REFRESH_TIME * globalvy;
                    //qDebug() << "x" << x << "y" << y << "yellow.x" << yellow.x << "yellow.y" << yellow.y << "endx" << endx << "endy" << endy;
                    //qDebug() << "the dis" <<distanceofPointandLine(x, y, yellow.x, yellow.y, endx, endy);
                    //qDebug() << "INFLATION_RADIUS" << PARAMS::OBSTACLE::INFLATION_RADIUS;
                    if(distanceofPointandLine(x, y, yellow.x, yellow.y, endx, endy) <= PARAMS::OBSTACLE::INFLATION_RADIUS)
                    {
                        //qDebug() << "true";
                        return true;
                    }
                    else
                    {
                        //qDebug() << "false";
                    }
                }
            }
        }
        break;
    default:
        break;
    }
    //qDebug() << "there is false finally!!!!!!";
    return false;
}

bool Obstacles::hasObstacle(double x1, double y1, double x2, double y2, int type)
{
    double distance = sqrt(pow(y1 - y2, 2) + pow(x1 - x2, 2));
    double theta = atan2(y2 - y1, x2 - x1);
    double dx = PARAMS::OBSTACLE::STEP_SIZE*cos(theta);
    double dy = PARAMS::OBSTACLE::STEP_SIZE*sin(theta);
    double x = x1;
    double y = y1;
        for(int i=0; i<distance/PARAMS::OBSTACLE::STEP_SIZE; i++){
            if(hasObstacle(x, y, type))
                return true;
            x += dx;
            y += dy;
        }
    return false;
}

double Obstacles::distanceofPointandLine(double pointx, double pointy, double startx, double starty, double endx, double endy)
{
    if (startx == endx && starty == endy)
        return sqrt(pow(endx-pointx,2)+pow(endy-pointy,2));
    double r = ((pointx-startx)*(endx-startx) + (pointy-starty)*(endy-starty))/(pow(endx-startx,2)+pow(endy-starty,2));
    //qDebug() << r;
    if(r <= 0) return sqrt(pow(startx-pointx,2)+pow(starty-pointy,2));
    else if(r >= 1) return sqrt(pow(endx-pointx,2)+pow(endy-pointy,2));
    else{
        //通过斜率求解
        if(endx == startx) return (pointx-startx);
        double k =  (endy-starty) / (endx-startx);
        return fabs(starty-pointy - k*(startx-pointx))/sqrt(k*k+1);
    }
}
