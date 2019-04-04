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

bool updateRRT()
{
    bool update = false;
    // have arrived && target change
    if(LocalPlanner::instance()->pathSize() == 0)
        update = true;
    // meet obstacles
    RobotInfo& me = MyDataManager::instance()->ourRobot();
    if(LocalPlanner::instance()->pathSize() != 0){
        MyPoint target = LocalPlanner::instance()->path.front();
        if(ObstaclesInfo::instance()->hasObstacle(me.x, me.y, target.x(), target.y(), CIRCLE))
            update = true;
    }
    // 到了下一个点重新更新

    // can reach my goal directly
    MyPoint goal = MyDataManager::instance()->goals.front();
    if(!ObstaclesInfo::instance()->hasObstacle(me.x, me.y, goal.x(), goal.y(), CIRCLE))
        update = true;

    return update;
}


void pathPlanning()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, MyDataManager::instance()->goals.front().x(), MyDataManager::instance()->goals.front().y());
    LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
//        if(updateRRT()){
            RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, MyDataManager::instance()->goals.front().x(), MyDataManager::instance()->goals.front().y());
            LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
            qDebug() << "path planning!!!";
//        }
    }
}

void debugMsg(){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//        qDebug() << "Nodelist size: " << RRTPlanner::instance()->NodeList.size();
//        VisualModule::instance()->drawTree(RRTPlanner::instance()->NodeList);
        VisualModule::instance()->drawLines(RRTPlanner::instance()->smoothPath);
    }
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    UDPReceiver::instance();

    bool USE_POTENTIAL = false;

    serialSender serial;
    // open serial
    if(!PARAMS::IS_SIMULATION){
//        serial.sendStartPacket();
        serial.openSerialPort();
    }

    // set Goals
    std::deque<MyPoint> goals = { MyPoint(200, 0), MyPoint(0, 100),  MyPoint(-200, 0)}; // -300是边界
    MyDataManager::instance()->setGoals(goals);

    std::thread* _thread1 = new std::thread([ = ] {pathPlanning();});
    std::thread* _thread2 = new std::thread([ = ] {debugMsg();});

    // vel sending
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

//        USE_POTENTIAL = ApPlanner::instance()->plan(MyDataManager::instance()->goals.front());

        if(!USE_POTENTIAL || true){
            // change goals
            if (PARAMS::DEBUG::kShowPathPlanner){
                std::cout << "[main.cpp] use RRT" << std::endl;
            }
            if(LocalPlanner::instance()->hasArrived(MyDataManager::instance()->goals.front())){
                MyDataManager::instance()->goals.push_back(MyDataManager::instance()->goals.front());
                MyDataManager::instance()->goals.pop_front();
                LocalPlanner::instance()->stopMoving();
                LocalPlanner::instance()->clearPath();
                qDebug() << "Change Goal to " << MyDataManager::instance()->goals.front().x() << MyDataManager::instance()->goals.front().y();
            }
            // velocity planning
            LocalPlanner::instance()->plan();
            // send command
            if(PARAMS::IS_SIMULATION)
                CommandSender::instance()->sendToSim(PARAMS::our_id, LocalPlanner::instance()->velX, LocalPlanner::instance()->velY, LocalPlanner::instance()->velW);
            else
    //            serial.sendToReal(2,100,0,0);
                serial.sendToReal(PARAMS::our_id, 30*LocalPlanner::instance()->velX, 0, -40*LocalPlanner::instance()->velW);
    //        qDebug() << "vel: "<< LocalPlanner::instance()->velX << LocalPlanner::instance()->velW;
        }
        else{
            if (PARAMS::DEBUG::kShowPathPlanner){
                std::cout << "[main.cpp] use artifical potential" << std::endl;
            }
            if ( PARAMS::IS_SIMULATION ){
                 CommandSender::instance()->sendToSim(PARAMS::our_id,
                                                      ApPlanner::instance()->v_x / 1000,
                                                      ApPlanner::instance()->v_y / 1000,
                                                      ApPlanner::instance()->v_w);
//                 CommandSender::instance()->sendToSim(PARAMS::our_id,
//                                                      1,
//                                                      1,
//                                                      0);
            }
            else {
                serial.sendToReal(2,
                                  ApPlanner::instance()->v_x / 10,
                                  ApPlanner::instance()->v_y / 10,
                                  -40 * ApPlanner::instance()->v_w);
            }
        }
    }

    return a.exec();
}
