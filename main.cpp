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
#include "utils/visualizationmodule.h"
#include <thread>

serialSender serial;

void pathPlanning()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, MyDataManager::instance()->goals.front().x(), MyDataManager::instance()->goals.front().y());
    LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, MyDataManager::instance()->goals.front().x(), MyDataManager::instance()->goals.front().y());
        LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
//        qDebug() << "path planning!!!";
    }
}

void debugMsg(){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//        qDebug() << "Nodelist size: " << RRTPlanner::instance()->NodeList.size();
        VisualModule::instance()->drawTree(RRTPlanner::instance()->NodeList);
    }
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    UDPReceiver::instance();
    bool if_use_artifical_potential = false;
    // open serial
    if(!PARAMS::IS_SIMULATION)
        serial.openSerialPort();
    // set Goals
    std::deque<MyPoint> goals = { MyPoint(-200, 0), MyPoint(0, 100),  MyPoint(200, 0)}; // -300是边界
    MyDataManager::instance()->setGoals(goals);

    std::thread* _thread1 = new std::thread([ = ] {pathPlanning();});
    std::thread* _thread2 = new std::thread([ = ] {debugMsg();});

    // vel sending
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if_use_artifical_potential = ApPlanner::instance()->plan(MyDataManager::instance()->goals.front());
        if(!if_use_artifical_potential){
            if(LocalPlanner::instance()->hasArrived(MyDataManager::instance()->goals.front())){
                MyDataManager::instance()->goals.push_back(MyDataManager::instance()->goals.front());
                MyDataManager::instance()->goals.pop_front();
                LocalPlanner::instance()->stopMoving();
                LocalPlanner::instance()->clearPath();
                qDebug() << "Change Goal to " << MyDataManager::instance()->goals.front().x() << MyDataManager::instance()->goals.front().y();
            }

    //        qDebug() << MyDataManager::instance()->ourRobot().x << MyDataManager::instance()->ourRobot().y;
    //        qDebug() << MyDataManager::instance()->blueRobots[4].x << MyDataManager::instance()->blueRobots[4].y;
            LocalPlanner::instance()->plan();
    //        qDebug() <<MyDataManager::instance()->yellowRobots[2].x <<MyDataManager::instance()->yellowRobots[2].y;
            if(PARAMS::IS_SIMULATION)
                CommandSender::instance()->sendToSim(PARAMS::our_id, LocalPlanner::instance()->velX, LocalPlanner::instance()->velY, LocalPlanner::instance()->velW);
            else
    //            serial.sendToReal(2,100,0,0);
    //        qDebug() << MyDataManager::instance()->ourRobot().vel_x << MyDataManager::instance()->ourRobot().vel_y;
                serial.sendToReal(2, 30*LocalPlanner::instance()->velX, 0, -40*LocalPlanner::instance()->velW);
    //        qDebug() << "vel: "<< LocalPlanner::instance()->velX << LocalPlanner::instance()->velW;

        }
        else{
            serial.sendToReal(2, 70 * ApPlanner::instance()->v_x,
                              70 * ApPlanner::instance()->v_y,
                              -40 * ApPlanner::instance()->v_w);
        }
    }

    return a.exec();
}
