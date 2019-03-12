#include "udpreceiver.h"
#include "proto/vision_detection.pb.h"
#include "datamanager.h"

namespace {
    int port = 23333;
}

UDPReceiver::UDPReceiver(QObject* parent) : QObject(parent)
{
    receiver = new QUdpSocket();
    receiver->bind(QHostAddress::AnyIPv4, port, QUdpSocket::ShareAddress);
    connect(receiver, SIGNAL(readyRead()), this, SLOT(readDatagrams()), Qt::DirectConnection);
//     qDebug() << "I'm Receiver";
}

UDPReceiver::~UDPReceiver(){
    receiver->abort();
}

void UDPReceiver::readDatagrams(){
    while(receiver->hasPendingDatagrams()){
        QByteArray datagram;
        datagram.resize(receiver->pendingDatagramSize());
        receiver->readDatagram(datagram.data(), datagram.size());
        // qDebug() << "Receiving: " << datagram.data();
        qDebug() << "Receiving data!!!";

        // Parse from datagram
        Vision_DetectionFrame vision;
        vision.ParseFromArray(datagram, datagram.size());

        // Read ball info
        auto ball = vision.balls();
        if (ball.has_vel_x()) MyDataManager::instance()->ball.vel_x = ball.vel_x();
        else MyDataManager::instance()->ball.vel_x = 9999;
        if (ball.has_vel_y()) MyDataManager::instance()->ball.vel_y = ball.vel_y();
        else MyDataManager::instance()->ball.vel_y = 9999;
        MyDataManager::instance()->ball.x = ball.x();
        MyDataManager::instance()->ball.y = ball.y();
        MyDataManager::instance()->ball.valid = ball.valid();

        // reset
        MyDataManager::instance()->reset();

        // Read blue robot info
        int blue_size = vision.robots_blue_size();
        for (int i=0; i<blue_size; i++){
            int robot_id = vision.robots_blue(i).robot_id();          // need improve!!!
            MyDataManager::instance()->validBlueRobots[robot_id] = true;
            // qDebug() << "DDQ debuging";
            MyDataManager::instance()->blueRobots[robot_id].robot_id = robot_id;
            MyDataManager::instance()->blueRobots[robot_id].x = vision.robots_blue(i).x();
            MyDataManager::instance()->blueRobots[robot_id].y = vision.robots_blue(i).y();
            if (vision.robots_blue(i).has_orientation())
                MyDataManager::instance()->blueRobots[robot_id].orientation = vision.robots_blue(i).orientation();
            else
                MyDataManager::instance()->blueRobots[robot_id].orientation = 9999;
            if (vision.robots_blue(i).has_vel_x())
                MyDataManager::instance()->blueRobots[robot_id].vel_x = vision.robots_blue(i).vel_x();
            else
                MyDataManager::instance()->blueRobots[robot_id].vel_x = 9999;
            if (vision.robots_blue(i).has_vel_y())
                MyDataManager::instance()->blueRobots[robot_id].vel_y = vision.robots_blue(i).vel_y();
            else
                MyDataManager::instance()->blueRobots[robot_id].vel_y = 9999;
            if (vision.robots_blue(i).has_rotate_vel())
                MyDataManager::instance()->blueRobots[robot_id].rotate_vel = vision.robots_blue(i).rotate_vel();
            else
                MyDataManager::instance()->blueRobots[robot_id].rotate_vel = 9999;
        }

        // Read yellow robot info
        int yellow_size = vision.robots_yellow_size();
        for (int i=0; i<yellow_size; i++){
            int robot_id = vision.robots_yellow(i).robot_id();          // need improve!!!
            MyDataManager::instance()->validYellowRobots[robot_id] = true;
            MyDataManager::instance()->yellowRobots[robot_id].robot_id = robot_id;
            MyDataManager::instance()->yellowRobots[robot_id].x = vision.robots_yellow(i).x();
            MyDataManager::instance()->yellowRobots[robot_id].y = vision.robots_yellow(i).y();
            if (vision.robots_yellow(i).has_orientation())
                MyDataManager::instance()->yellowRobots[robot_id].orientation = vision.robots_yellow(i).orientation();
            else
                MyDataManager::instance()->yellowRobots[robot_id].orientation = 9999;
            if (vision.robots_yellow(i).has_vel_x())
                MyDataManager::instance()->yellowRobots[robot_id].vel_x = vision.robots_yellow(i).vel_x();
            else
                MyDataManager::instance()->yellowRobots[robot_id].vel_x = 9999;
            if (vision.robots_yellow(i).has_vel_y())
                MyDataManager::instance()->yellowRobots[robot_id].vel_y = vision.robots_yellow(i).vel_y();
            else
                MyDataManager::instance()->yellowRobots[robot_id].vel_y = 9999;
            if (vision.robots_yellow(i).has_rotate_vel())
                MyDataManager::instance()->yellowRobots[robot_id].rotate_vel = vision.robots_yellow(i).rotate_vel();
            else
                MyDataManager::instance()->yellowRobots[robot_id].rotate_vel = 9999;
        }

        // debug
//        for(int i=0; i<16; i++){
//            qDebug() << "robot_id: " << i << "  " << MyDataManager::instance()->validBlueRobots[i];
//            qDebug() << "robot_id: " << i << "  " << MyDataManager::instance()->validYellowRobots[i];
//            qDebug() << "robot pos" << i << "  " << MyDataManager::instance()->blueRobots[i].x << "  " << MyDataManager::instance()->blueRobots[i].y;
//        }
    }
}
