#include "visualizationmodule.h"
#include "proto/zss_debug.pb.h"
#include "proto/grSim_Packet.pb.h"
#include "utils/params.h"
#include "utils/mymath.h"
#include "utils/datamanager.h"
#include "algorithm/rrt.h"


visualizationmodule::visualizationmodule(QObject *parent) : QObject(parent)
{
    sender = new QUdpSocket();
}

visualizationmodule::~visualizationmodule()
{
    sender->abort();
}

void visualizationmodule::drawPoint(MyPoint point)
{
    ZSS::Protocol::Debug_Msgs msgs;
    ZSS::Protocol::Debug_Msg* msg;
    ZSS::Protocol::Debug_Line* line;
    msg = msgs.add_msgs();
    msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
    msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
    line = msg->mutable_line();
    line->mutable_start()->set_x(point.x()-5);
    line->mutable_start()->set_y(point.y()-5);
    line->mutable_end()->set_x(point.x()+5);
    line->mutable_end()->set_y(point.y()+5);
    line->set_forward(false);
    line->set_back(false);
    msg = msgs.add_msgs();
    msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
    msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
    line = msg->mutable_line();
    line->mutable_start()->set_x(point.x()-5);
    line->mutable_start()->set_y(point.y()+5);
    line->mutable_end()->set_x(point.x()+5);
    line->mutable_end()->set_y(point.y()-5);
    line->set_forward(false);
    line->set_back(false);

    int msgs_size = msgs.ByteSize();
    QByteArray msgs_data(msgs_size, 0);
    msgs.SerializeToArray(msgs_data.data(), msgs_data.size());
    sender->writeDatagram(msgs_data, msgs_size, PARAMS::visualAddress, PARAMS::visualPort);
}

void visualizationmodule::drawPoints(std::vector<MyPoint> &somePoints)
{
    ZSS::Protocol::Debug_Msgs msgs;
    ZSS::Protocol::Debug_Msg* msg;
    ZSS::Protocol::Debug_Line* line;
    for(unsigned i = 0; i < somePoints.size(); i++)
    {
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(somePoints[i].x()-5);
        line->mutable_start()->set_y(somePoints[i].y()-5);
        line->mutable_end()->set_x(somePoints[i].x()+5);
        line->mutable_end()->set_y(somePoints[i].y()+5);
        line->set_forward(false);
        line->set_back(false);
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(somePoints[i].x()-5);
        line->mutable_start()->set_y(somePoints[i].y()+5);
        line->mutable_end()->set_x(somePoints[i].x()+5);
        line->mutable_end()->set_y(somePoints[i].y()-5);
        line->set_forward(false);
        line->set_back(false);
    }

    int msgs_size = msgs.ByteSize();
    QByteArray msgs_data(msgs_size, 0);
    msgs.SerializeToArray(msgs_data.data(), msgs_data.size());
    sender->writeDatagram(msgs_data, msgs_size, PARAMS::visualAddress, PARAMS::visualPort);
}

void visualizationmodule::drawLines(std::vector<MyPoint> &somePoints)
{
    ZSS::Protocol::Debug_Msgs msgs;
    ZSS::Protocol::Debug_Msg* msg;
    ZSS::Protocol::Debug_Line* line;
    for(unsigned i = 1; i < somePoints.size(); i++)
    {
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(somePoints[i-1].x());
        line->mutable_start()->set_y(somePoints[i-1].y());
        line->mutable_end()->set_x(somePoints[i].x());
        line->mutable_end()->set_y(somePoints[i].y());
        line->set_forward(false);
        line->set_back(false);
    }

    int msgs_size = msgs.ByteSize();
    QByteArray msgs_data(msgs_size, 0);
    msgs.SerializeToArray(msgs_data.data(), msgs_data.size());
    sender->writeDatagram(msgs_data, msgs_size, PARAMS::visualAddress, PARAMS::visualPort);
//    sender->writeDatagram(msgs_data, msgs_size,  QHostAddress("127.0.0.1"), 20001);
}

void visualizationmodule::drawTree(std::vector<Node> &someNodes)
{
    ZSS::Protocol::Debug_Msgs msgs;
    ZSS::Protocol::Debug_Msg* msg;
    ZSS::Protocol::Debug_Line* line;
    for(unsigned i = 0; i < someNodes.size(); i++)
    {
        //画出点来
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(someNodes[i].x-5);
        line->mutable_start()->set_y(someNodes[i].y-5);
        line->mutable_end()->set_x(someNodes[i].x+5);
        line->mutable_end()->set_y(someNodes[i].y+5);
        line->set_forward(false);
        line->set_back(false);
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(someNodes[i].x+5);
        line->mutable_start()->set_y(someNodes[i].y-5);
        line->mutable_end()->set_x(someNodes[i].x-5);
        line->mutable_end()->set_y(someNodes[i].y+5);
        line->set_forward(false);
        line->set_back(false);
        //画线
        if(someNodes[i].parent == -1) continue;
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_BLUE);
        line = msg->mutable_line();
        line->mutable_start()->set_x(someNodes[i].x);
        line->mutable_start()->set_y(someNodes[i].y);
        line->mutable_end()->set_x(someNodes[someNodes[i].parent].x);
        line->mutable_end()->set_y(someNodes[someNodes[i].parent].y);
        line->set_forward(false);
        line->set_back(false);
    }
    //画一个特别的终点
    for(int j = 0; j < 15; j++)
    {
        int i = someNodes.size()-1;
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(someNodes[i].x-j);
        line->mutable_start()->set_y(someNodes[i].y-15+j);
        line->mutable_end()->set_x(someNodes[i].x+j);
        line->mutable_end()->set_y(someNodes[i].y+15-j);
        line->set_forward(false);
        line->set_back(false);
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(someNodes[i].x+15-j);
        line->mutable_start()->set_y(someNodes[i].y-j);
        line->mutable_end()->set_x(someNodes[i].x-15+j);
        line->mutable_end()->set_y(someNodes[i].y+j);
        line->set_forward(false);
        line->set_back(false);
    }
    //发送信息
    int msgs_size = msgs.ByteSize();
    QByteArray msgs_data(msgs_size, 0);
    msgs.SerializeToArray(msgs_data.data(), msgs_data.size());
    sender->writeDatagram(msgs_data, msgs_size, PARAMS::visualAddress, PARAMS::visualPort);
//    sender->writeDatagram(msgs_data, msgs_size,  QHostAddress("127.0.0.1"), 20001);
}

void visualizationmodule::drawAll(std::deque<MyPoint> goals)
{
    ZSS::Protocol::Debug_Msgs msgs;
    ZSS::Protocol::Debug_Msg* msg;
    ZSS::Protocol::Debug_Line* line;
    std::vector<Node> someNodes = RRTPlanner::instance()->NodeList;
    for(unsigned i = 0; i < someNodes.size(); i++)
    {
        //画出点来
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(someNodes[i].x-5);
        line->mutable_start()->set_y(someNodes[i].y-5);
        line->mutable_end()->set_x(someNodes[i].x+5);
        line->mutable_end()->set_y(someNodes[i].y+5);
        line->set_forward(false);
        line->set_back(false);
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(someNodes[i].x+5);
        line->mutable_start()->set_y(someNodes[i].y-5);
        line->mutable_end()->set_x(someNodes[i].x-5);
        line->mutable_end()->set_y(someNodes[i].y+5);
        line->set_forward(false);
        line->set_back(false);
        //画线
        if(someNodes[i].parent == -1) continue;
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_BLUE);
        line = msg->mutable_line();
        line->mutable_start()->set_x(someNodes[i].x);
        line->mutable_start()->set_y(someNodes[i].y);
        line->mutable_end()->set_x(someNodes[someNodes[i].parent].x);
        line->mutable_end()->set_y(someNodes[someNodes[i].parent].y);
        line->set_forward(false);
        line->set_back(false);
    }
    //画几个特别的终点
    for(int i = 0; i < goals.size(); i++)
    {
        for(int j = 0; j < 15; j++)
        {
            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
            line = msg->mutable_line();
            line->mutable_start()->set_x(goals[i].x()-j);
            line->mutable_start()->set_y(goals[i].y()-15+j);
            line->mutable_end()->set_x(goals[i].x()+j);
            line->mutable_end()->set_y(goals[i].y()+15-j);
            line->set_forward(false);
            line->set_back(false);
            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
            line = msg->mutable_line();
            line->mutable_start()->set_x(goals[i].x()+15-j);
            line->mutable_start()->set_y(goals[i].y()-j);
            line->mutable_end()->set_x(goals[i].x()-15+j);
            line->mutable_end()->set_y(goals[i].y()+j);
            line->set_forward(false);
            line->set_back(false);
            msg = msgs.add_msgs();
            //这里是要写个字
//            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_TEXT);
//            msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
//            text = msg->mutable_text();
//            text->set_text(std::to_string(i));
//            text->
        }
    }
    //画平滑的线
    std::vector<MyPoint> somePoints = RRTPlanner::instance()->smoothPath;
    for(unsigned i = 1; i < somePoints.size(); i++)
    {
        msg = msgs.add_msgs();
        msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
        msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
        line = msg->mutable_line();
        line->mutable_start()->set_x(somePoints[i-1].x());
        line->mutable_start()->set_y(somePoints[i-1].y());
        line->mutable_end()->set_x(somePoints[i].x());
        line->mutable_end()->set_y(somePoints[i].y());
        line->set_forward(false);
        line->set_back(false);
    }
    int msgs_size = msgs.ByteSize();
    QByteArray msgs_data(msgs_size, 0);
    msgs.SerializeToArray(msgs_data.data(), msgs_data.size());
    sender->writeDatagram(msgs_data, msgs_size, PARAMS::visualAddress, PARAMS::visualPort);
}
