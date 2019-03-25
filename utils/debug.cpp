#include "debug.h"
#include "proto/zss_debug.pb.h"
#include "proto/grSim_Packet.pb.h"
#include "utils/params.h"
#include "utils/mymath.h"

Debug::Debug(QObject *parent) : QObject(parent)
{
    sender = new QUdpSocket();
}

Debug::~Debug()
{
    sender->abort();
}

void Debug::sendToSim(std::vector<MyPoint> &somepoints)
{
    ZSS::Protocol::Debug_Msgs msgs;
    ZSS::Protocol::Debug_Msg* msg;
    ZSS::Protocol::Debug_Line* line;
    for(int i = 0; i < somepoints.size(); i++)
    {
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(somepoints[i].x()-5);
        line->mutable_start()->set_y(somepoints[i].y()-5);
        line->mutable_end()->set_x(somepoints[i].x()+5);
        line->mutable_end()->set_y(somepoints[i].y()+5);
        line->set_forward(false);
        line->set_back(false);
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(somepoints[i].x()-5);
        line->mutable_start()->set_y(somepoints[i].y()+5);
        line->mutable_end()->set_x(somepoints[i].x()+5);
        line->mutable_end()->set_y(somepoints[i].y()-5);
        line->set_forward(false);
        line->set_back(false);
    }
//    ZSS::Protocol::Debug_Msg* msg = msgs.add_msgs();
//    msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
//    msg->set_color(ZSS::Protocol::Debug_Msg_Color_BLUE);
//    ZSS::Protocol::Debug_Line* line = msg->mutable_line();
//    line->mutable_start()->set_x(0);
//    line->mutable_start()->set_y(0);
//    line->mutable_end()->set_x(100);
//    line->mutable_end()->set_y(100);
//    line->set_forward(true);
//    line->set_back(false);
    int msgs_size = msgs.ByteSize();
    QByteArray msgs_data(msgs_size, 0);
    msgs.SerializeToArray(msgs_data.data(), msgs_data.size());
    sender->writeDatagram(msgs_data, msgs_size, QHostAddress("127.0.0.1"), 20001);
}

void Debug::sendToSim(std::vector<Node> &someNodes)
{
    //
}