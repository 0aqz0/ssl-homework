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
#include "utils/paramloader.h"
#include <thread>
#include <iostream>

serialSender serial;
PathPlanner localPlanner;

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
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        localPlanner.plan();
        if(PARAMS::IS_SIMULATION)
            CommandSender::instance()->sendToSim(PARAMS::our_id, localPlanner.velX, -localPlanner.velY, localPlanner.velW);
        else
            // 从1开始
//                serial.sendToReal(PARAMS::our_id, 180 * localPlanner.velX, 180 * localPlanner.velY, -40*localPlanner.velW);
            serial.sendToReal(PARAMS::our_id, 35 * localPlanner.velX, 50 * localPlanner.velY, -10*localPlanner.velW);
//        qDebug() << "vel: "<< localPlanner.velX << localPlanner.velY << localPlanner.velW;
    }

    return a.exec();
}
