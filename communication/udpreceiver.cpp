#include "udpreceiver.h"
#include "proto/vision_detection.pb.h"
#include "utils/datamanager.h"
#include "utils/params.h"
#include "algorithm/pathplanner.h"
#include "udpsender.h"
#include "algorithm/rrt.h"

extern serialSender serial;

UDPReceiver::UDPReceiver(QObject* parent) : QObject(parent)
{
    receiver = new QUdpSocket();
    receiver->bind(QHostAddress::AnyIPv4, PARAMS::visionPort, QUdpSocket::ShareAddress);
    if(!PARAMS::IS_SIMULATION)
        receiver->joinMulticastGroup(QHostAddress("224.5.23.2"));
    connect(receiver, SIGNAL(readyRead()), this, SLOT(readDatagrams()), Qt::DirectConnection);
}

UDPReceiver::~UDPReceiver(){
    receiver->abort();
}

void UDPReceiver::readDatagrams(){
    while(receiver->hasPendingDatagrams()){
        QByteArray datagram;
        datagram.resize(receiver->pendingDatagramSize());
        receiver->readDatagram(datagram.data(), datagram.size());
        // qDebug() << "Receiving data!!!";

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
            MyDataManager::instance()->blueRobots[robot_id].x = vision.robots_blue(i).x()/10;
            MyDataManager::instance()->blueRobots[robot_id].y = -vision.robots_blue(i).y()/10;
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
            MyDataManager::instance()->yellowRobots[robot_id].x = vision.robots_yellow(i).x()/10;
            MyDataManager::instance()->yellowRobots[robot_id].y = -vision.robots_yellow(i).y()/10;
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

        // vel sending
        if(LocalPlanner::instance()->hasArrived(MyDataManager::instance()->goals.front())){
            MyDataManager::instance()->goals.push_back(MyDataManager::instance()->goals.front());
            MyDataManager::instance()->goals.pop_front();
            LocalPlanner::instance()->stopMoving();
            LocalPlanner::instance()->clearPath();
            qDebug() << "Change Goal to " << MyDataManager::instance()->goals.front().x() << MyDataManager::instance()->goals.front().y();
        }
        LocalPlanner::instance()->plan();
//        qDebug() <<MyDataManager::instance()->yellowRobots[2].x <<MyDataManager::instance()->yellowRobots[2].y;
        if(PARAMS::IS_SIMULATION)
            CommandSender::instance()->sendToSim(PARAMS::our_id, LocalPlanner::instance()->velX, LocalPlanner::instance()->velY, LocalPlanner::instance()->velW);
        else
//            serial.sendToReal(2, 0, 0, -50*LocalPlanner::instance()->velW);
            serial.sendToReal(2, 30*LocalPlanner::instance()->velX, 0, -40*LocalPlanner::instance()->velW);
        qDebug() << "vel: "<< LocalPlanner::instance()->velX << LocalPlanner::instance()->velW;
    }
}
