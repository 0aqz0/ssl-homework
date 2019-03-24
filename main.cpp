#include <QCoreApplication>
#include "communication/udpreceiver.h"
#include "communication/udpsender.h"
#include "communication/serialsender.h"
#include "utils/datamanager.h"
#include <thread>
#include "utils/mymath.h"
#include <iostream>

void loop()
{
//    RealCommandSender::instance()->openSerialPort();
//    RealCommandSender::instance()->sendStartPacket();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        qDebug() << "in the thread!";
//        RealCommandSender::instance()->sendToReal(0, 100, 0, 0);
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
