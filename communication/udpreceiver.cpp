#include "udpreceiver.h"
#include "proto/vision_detection.pb.h"
#include "utils/datamanager.h"
#include "utils/params.h"
#include "udpsender.h"
#include "algorithm/rrt.h"
#include "algorithm/artifical_potential.h"
#include <thread>

UDPReceiver* UDPReceiver::_instance = 0;

namespace {
    std::thread* _thread = nullptr;
}

UDPReceiver::UDPReceiver()
{
    receiver = new QUdpSocket();
    receiver->bind(QHostAddress::AnyIPv4, PARAMS::visionPort, QUdpSocket::ShareAddress);
    if(!PARAMS::IS_SIMULATION)
        receiver->joinMulticastGroup(QHostAddress("224.5.23.2"));
    _thread = new std::thread([ = ] {readDatagrams();});
}

UDPReceiver::~UDPReceiver(){
    receiver->abort();
    delete _thread;
}

UDPReceiver* UDPReceiver::instance(){
    if(_instance == 0)
        _instance = new UDPReceiver();
    return _instance;
}

void UDPReceiver::readDatagrams(){
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        while(receiver->hasPendingDatagrams()){
            QByteArray datagram;
            datagram.resize(receiver->pendingDatagramSize());
            receiver->readDatagram(datagram.data(), datagram.size());
//            qDebug() << "Receiving data!!!";

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
                    MyDataManager::instance()->blueRobots[robot_id].vel_x = 0;
                if (vision.robots_blue(i).has_vel_y())
                    MyDataManager::instance()->blueRobots[robot_id].vel_y = vision.robots_blue(i).vel_y();
                else
                    MyDataManager::instance()->blueRobots[robot_id].vel_y = 0;
                if (vision.robots_blue(i).has_rotate_vel())
                    MyDataManager::instance()->blueRobots[robot_id].rotate_vel = vision.robots_blue(i).rotate_vel();
                else
                    MyDataManager::instance()->blueRobots[robot_id].rotate_vel = 0;
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
                    MyDataManager::instance()->yellowRobots[robot_id].vel_x = 0;
                if (vision.robots_yellow(i).has_vel_y())
                    MyDataManager::instance()->yellowRobots[robot_id].vel_y = vision.robots_yellow(i).vel_y();
                else
                    MyDataManager::instance()->yellowRobots[robot_id].vel_y = 0;
                if (vision.robots_yellow(i).has_rotate_vel())
                    MyDataManager::instance()->yellowRobots[robot_id].rotate_vel = vision.robots_yellow(i).rotate_vel();
                else
                    MyDataManager::instance()->yellowRobots[robot_id].rotate_vel = 0;
            }

            // Update cycle
            MyDataManager::instance()->updateCycle();
        }
    }
}
