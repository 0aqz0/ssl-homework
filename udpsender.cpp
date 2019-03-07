#include "udpsender.h"
#include "grSim_Packet.pb.h"

namespace {
    int port = 23333;
}

UDPSender::UDPSender(QObject *parent) : QObject(parent)
{
    sender = new QUdpSocket();
}

UDPSender::~UDPSender()
{
    sender->abort();
}

void UDPSender::sendCommand()
{
    grSim_Packet packet;
    auto* command = packet.mutable_commands();
    command->set_timestamp(0);
    command->set_isteamyellow(false);
    auto* robot_command = command->add_robot_commands();
    robot_command->set_id(0);
    robot_command->set_kickspeedx(0);
    robot_command->set_kickspeedz(0);
    robot_command->set_veltangent(0);
    robot_command->set_velnormal(0);
    robot_command->set_velangular(5);
    robot_command->set_spinner(0);
    robot_command->set_wheelsspeed(false);
    int size = packet.ByteSize();
    QByteArray data(size, 0);
    packet.SerializePartialToArray(data.data(), data.size());
    sender->writeDatagram(data, size, QHostAddress("127.0.0.1"), 20011);
}
