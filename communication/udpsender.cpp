#include "udpsender.h"
#include "proto/grSim_Packet.pb.h"
#include "proto/zss_debug.pb.h"
#include "utils/params.h"

UDPSender::UDPSender(QObject *parent) : QObject(parent)
{
    sender = new QUdpSocket();
}

UDPSender::~UDPSender()
{
    sender->abort();
}

void UDPSender::sendToSim(int robot_id, double vel_x, double vel_y, double vel_w)
{
    grSim_Packet packet;
    auto* command = packet.mutable_commands();
    command->set_timestamp(0);
    if(PARAMS::isBlue)
        command->set_isteamyellow(false);
    else
        command->set_isteamyellow(true);
    auto* robot_command = command->add_robot_commands();
    robot_command->set_id(robot_id-1);
    robot_command->set_kickspeedx(0);
    robot_command->set_kickspeedz(0);
    robot_command->set_veltangent(vel_x);
    robot_command->set_velnormal(vel_y);
    robot_command->set_velangular(vel_w);
    robot_command->set_spinner(0);
    robot_command->set_wheelsspeed(false);
    int size = packet.ByteSize();
    QByteArray data(size, 0);
    packet.SerializePartialToArray(data.data(), data.size());
    sender->writeDatagram(data, size, PARAMS::simAddress, PARAMS::simPort);
}
