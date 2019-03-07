#include "udpreceiver.h"
#include "vision_detection.pb.h"

UDPReceiver::UDPReceiver(QObject* parent) : QObject(parent)
{
    receiver = new QUdpSocket();
    receiver->bind(QHostAddress::LocalHost, 23333);
    connect(receiver, SIGNAL(readyRead()), this, SLOT(readDatagrams()));
    // qDebug() << "I'm Receiver";
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

        // Parse from datagram
        Vision_DetectionFrame vision;
        vision.ParseFromArray(datagram, datagram.size());
        auto ball = vision.balls();
        qDebug() << "ball pos_x: " << ball.x() << "    ball pos_y: " << ball.y();
    }
}
