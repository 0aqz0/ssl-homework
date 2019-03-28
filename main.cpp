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
#include <iostream>

serialSender serial;

void pathPlanning()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, MyDataManager::instance()->goals.front().x(), MyDataManager::instance()->goals.front().y());
    LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
//    ApPlanner::instance()->plan( MyDataManager::instance()->goals.front() );
    qDebug() << "path planning!!!";
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, MyDataManager::instance()->goals.front().x(), MyDataManager::instance()->goals.front().y());
//        LocalPlanner::instance()->mutex.lock();
        LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
        VisualModule::instance()->drawTree(RRTPlanner::instance()->NodeList);
        for(int i=0; i<RRTPlanner::instance()->smoothPath.size(); i++){
            std::cout << RRTPlanner::instance()->smoothPath[i].x() << ", " << RRTPlanner::instance()->smoothPath[i].y() << "    ";
        }
        std::cout << std::endl;
//        LocalPlanner::instance()->mutex.unlock();
//        ApPlanner::instance()->plan( MyDataManager::instance()->goals.front() );
        qDebug() << "path planning!!!";
    }
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    VisionReceiver::instance();
    // open serial
    if(!PARAMS::IS_SIMULATION)
        serial.openSerialPort();
    // set Goals
    std::deque<MyPoint> goals = { MyPoint(150, 0), MyPoint(0, 150), MyPoint(-150, 0), MyPoint(0, -150)};
    MyDataManager::instance()->setGoals(goals);

    std::thread* _thread1 = new std::thread([ = ] {pathPlanning();});

    return a.exec();
}
