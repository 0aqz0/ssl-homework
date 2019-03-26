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
    while(true)//好像同一时刻能画出一种形态？就是不能保留下来
    {
        VisualModule::instance()->drawLines(somepoints);
        VisualModule::instance()->drawPoints(somepoints);
    }

//    DebugMsgSender::instance()->drawLines(somepoints);
//    RealCommandSender::instance()->openSerialPort();
//    RealCommandSender::instance()->sendStartPacket();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x,MyDataManager::instance()->ourRobot().y, 300, 0);
    RRTStarPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x,MyDataManager::instance()->ourRobot().y, 300, 0);
    LocalPlanner::instance()->updatePath(RRTStarPlanner::instance()->finalPath);
    while(true){
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        qDebug() << "in the thread!";
//        RealCommandSender::instance()->sendToReal(0, 100, 0, 0);
        LocalPlanner::instance()->plan();
    }
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    VisionReceiver::instance();

    std::thread* _thread = new std::thread([ = ] {loop();});

//    MyVector b(3, 4);
//    MyVector c(3, 4);
//    std::cout << (b + c).x() << ", " << (b + c).y()
//              << std::endl;

    return a.exec();
}
