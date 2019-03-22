#include "udpsender.h"
#include "proto/grSim_Packet.pb.h"
#include "proto/zss_debug.pb.h"

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

void UDPSender::sendToSim(int robot_id, double vel_x, double vel_y, double vel_w)
{
    grSim_Packet packet;
    auto* command = packet.mutable_commands();
    command->set_timestamp(0);
    command->set_isteamyellow(false);
    auto* robot_command = command->add_robot_commands();
    robot_command->set_id(robot_id);
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
    sender->writeDatagram(data, size, QHostAddress("127.0.0.1"), 20011);

//    ZSS::Protocol::Debug_Msgs msgs;
//    ZSS::Protocol::Debug_Msg* msg = msgs.add_msgs();
//    msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
//    msg->set_color(ZSS::Protocol::Debug_Msg_Color_BLUE);
//    ZSS::Protocol::Debug_Line* line = msg->mutable_line();
//    line->mutable_start()->set_x(0);
//    line->mutable_start()->set_y(0);
//    line->mutable_end()->set_x(100);
//    line->mutable_end()->set_y(100);
//    line->set_forward(false);
//    line->set_back(false);
//    int msg_size = msgs.ByteSize();
//    QByteArray msg_data(msg_size, 0);
//    msgs.SerializeToArray(msg_data.data(), msg_data.size());
//    sender->writeDatagram(msg_data, msg_size, QHostAddress("127.0.0.1"), 20001);

}
