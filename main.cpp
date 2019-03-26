#include <QCoreApplication>
#include "communication/udpreceiver.h"
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "utils/datamanager.h"
#include "utils/mymath.h"
#include "algorithm/pathplanner.h"
#include "algorithm/rrt.h"
#include "algorithm/rrtstar.h"
#include "utils/visualizationmodule.h"
#include <thread>
#include <iostream>

void loop()
{
    std::vector<MyPoint> somepoints;
    somepoints.push_back(MyPoint(100,400));
    somepoints.push_back(MyPoint(400,100));
    somepoints.push_back(MyPoint(-200,200));
//    while(true)//好像同一时刻能画出一种形态？就是不能保留下来
//    {
//        VisualModule::instance()->drawLines(somepoints);
//        VisualModule::instance()->drawPoints(somepoints);
//    }

//    DebugMsgSender::instance()->drawLines(somepoints);
//    RealCommandSender::instance()->openSerialPort();
//    RealCommandSender::instance()->sendStartPacket();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x,MyDataManager::instance()->ourRobot().y, 500, 0);
    LocalPlanner::instance()->updatePath(RRTPlanner::instance()->smoothPath);
//    RRTStarPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x,MyDataManager::instance()->ourRobot().y, 300, 0);
//    LocalPlanner::instance()->updatePath(RRTStarPlanner::instance()->finalPath);
    while(true){
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        qDebug() << "in the thread!";
//        RealCommandSender::instance()->sendToReal(0, 100, 0, 0);
        LocalPlanner::instance()->plan();
//        VisualModule::instance()->drawLines(RRTStarPlanner::instance()->finalPath);
//        VisualModule::instance()->drawLines(RRTPlanner::instance()->finalPath);
        VisualModule::instance()->drawLines(RRTPlanner::instance()->smoothPath);
    }
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    VisionReceiver::instance();

    std::thread* _thread = new std::thread([ = ] {loop();});

    MyPoint p1(4, 6);
    MyPoint p2(2, 3);
    MyVector v1 = p2 - p1;
    std::cout << "[main.cpp] v1: " << v1.x() << ", "
              << v1.y() << std::endl;

    return a.exec();
}
