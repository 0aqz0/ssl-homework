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
//    RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, MyDataManager::instance()->goals.front().x(), MyDataManager::instance()->goals.front().y());
//    LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
    ApPlanner::instance()->plan( MyDataManager::instance()->goals.front() );
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
//        RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x, MyDataManager::instance()->ourRobot().y, MyDataManager::instance()->goals.front().x(), MyDataManager::instance()->goals.front().y());
//        LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
        ApPlanner::instance()->plan( MyDataManager::instance()->goals.front() );
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
    VisionReceiver::instance();
    // open serial
    if(!PARAMS::IS_SIMULATION)
        serial.openSerialPort();
    // set Goals
    std::deque<MyPoint> goals = { MyPoint(-200, 0), MyPoint(0, 150),  MyPoint(200, 0)}; // -300是边界
    MyDataManager::instance()->setGoals(goals);

    std::thread* _thread1 = new std::thread([ = ] {pathPlanning();});
    std::thread* _thread2 = new std::thread([ = ] {debugMsg();});

    return a.exec();
}
