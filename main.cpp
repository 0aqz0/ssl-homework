#include <QCoreApplication>
#include "communication/udpreceiver.h"
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "utils/datamanager.h"
#include "utils/mymath.h"
#include "algorithm/pathplanner.h"
#include "algorithm/rrt.h"
#include <thread>
#include <iostream>

void loop()
{
//    RealCommandSender::instance()->openSerialPort();
//    RealCommandSender::instance()->sendStartPacket();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RRTPlanner::instance()->plan(MyDataManager::instance()->ourRobot().x,MyDataManager::instance()->ourRobot().y, 300, 0);
    while(true){
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        qDebug() << "in the thread!";
//        RealCommandSender::instance()->sendToReal(0, 100, 0, 0);
//        Planner::instance()->plan();
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
