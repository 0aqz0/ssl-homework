#include <QCoreApplication>
#include "udpreceiver.h"
#include "udpsender.h"
#include "mymath.h"
#include <iostream>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    VisionReceiver::instance();

    MyPoint p1(4, 6);
    MyPoint p2(2, 3);
    MyVector v1 = p2 - p1;
    std::cout << "[main.cpp] v1: " << v1.x() << ", "
              << v1.y() << std::endl;

    return a.exec();
}
