#include <QCoreApplication>
#include "communication/udpreceiver.h"
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "utils/datamanager.h"
#include "utils/mymath.h"
#include "algorithm/pathplanner.h"
#include "algorithm/rrt.h"
#include "algorithm/rrtstar.h"
#include "algorithm/artifical_potential.h"
#include "algorithm/obstacles.h"
#include "utils/visualizationmodule.h"
#include <thread>
#include <iostream>

serialSender serial;
PathPlanner localPlanner;

// set Goals
std::deque<MyPoint> goals = { MyPoint(-200, -100), MyPoint(200, 100)}; // -300是边界

bool updateRRT()
{
    bool update = false;
    // have arrived && target change && no path
    if(localPlanner.pathSize() == 0)
        update = true;
    // meet obstacles
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    if(localPlanner.pathSize() != 0){
        MyPoint target = localPlanner.path.front();
        if(ObstaclesInfo::instance()->hasObstacle(me.x, me.y, target.x(), target.y(), PARAMS::OBSTACLE::OBSTACLETYPE))
            update = true;
    }
    // 到了下一个点重新更新

    // can reach my goal directly
    MyPoint goal = goals.front();
    if(!ObstaclesInfo::instance()->hasObstacle(me.x, me.y, goal.x(), goal.y(), PARAMS::OBSTACLE::OBSTACLETYPE))
        update = true;

    return update;
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    UDPReceiver::instance();

    // open serial
    if(!PARAMS::IS_SIMULATION){
//        serial.sendStartPacket();
        serial.openSerialPort();
    }

    // vel sending
    while(true){
        if ( PARAMS::DEBUG::kMainDebug ){
            std::cout << "[main.cpp] send velocity" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        bool if_use_artifical = ApPlanner::instance()->plan(goals.front());
        if ( !if_use_artifical || true){
            // update RRT
            if(updateRRT()){
//                qDebug() << "path planning!";
                RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, goals.front().x(), goals.front().y());
                localPlanner.updatePath(RRTPlanner::instance()->smoothPath);
            }
            // change goals
            if(localPlanner.hasArrived(goals.front())){
                goals.push_back(goals.front());
                goals.pop_front();
                localPlanner.stopMoving();
                localPlanner.clearPath();
//                qDebug() << "Change Goal to " << goals.front().x() << goals.front().y();
            }
            localPlanner.plan();
            if(PARAMS::IS_SIMULATION)
                CommandSender::instance()->sendToSim(PARAMS::our_id, localPlanner.velX, -localPlanner.velY, localPlanner.velW);
            else
                // 从1开始
//                serial.sendToReal(PARAMS::our_id, 180 * localPlanner.velX, 180 * localPlanner.velY, -40*localPlanner.velW);
                serial.sendToReal(PARAMS::our_id, 140 * localPlanner.velX, 200 * localPlanner.velY, 0);
//                 qDebug() << "vel: "<< localPlanner.velX << localPlanner.velY << localPlanner.velW;
                // qDebug() << MyDataManager::instance()->ourRobot().x << MyDataManager::instance()->ourRobot().y;
        }
        else {
            if(PARAMS::IS_SIMULATION){
                 CommandSender::instance()->sendToSim(PARAMS::our_id,
                                                      ApPlanner::instance()->v_x / 1000,
                                                      ApPlanner::instance()->v_y / 1000,
                                                      ApPlanner::instance()->v_w);
            }
            else{
                serial.sendToReal(PARAMS::our_id,
                                  ApPlanner::instance()->v_x / 10,
                                  ApPlanner::instance()->v_y / 10,
                                  ApPlanner::instance()->v_w);
            }
        }
        if ( !if_use_artifical )
        {
            VisualModule::instance()->drawAll(goals);
//            VisualModule::instance()->drawLines(RRTPlanner::instance()->smoothPath);
        }
        else{
//            VisualModule::instance()->drawPoint(MyPoint(0, -0));
            VisualModule::instance()->drawAll(goals);
        }
    }
    return a.exec();
}
