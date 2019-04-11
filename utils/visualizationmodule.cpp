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
    for(unsigned i = 0; i < goals.size(); i++)
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

    //画出膨胀之后的障碍物来，这里只写了capsule，如果需要circle再说吧

    int x[8], y[8], maxdistance, mydis, first, second, third, forth;
    double globalvx, globalvy, theta, endx, endy;
    for(int i=0; i<PARAMS::ROBOT_NUM; i++){
        //为了debug
    //for(int i=7; i<8; i++){
        // blue robot
        //for(int k = 0; k < 8; k++) x[k] = y[k] = 0;
        if(MyDataManager::instance()->validBlueRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id-1) : i==(PARAMS::our_id-1)) && PARAMS::isBlue)){
            RobotInfo& blue = MyDataManager::instance()->blueRobots[i];
            //尤其注意，这里是路子豪写的，是按照正常情况下的X-Y坐标系来做的，对于仿真和实际的环境有点痴迷，详情请询问路子豪
            //globalvx = blue.vel_x * cos(blue.orientation) + blue.vel_y * sin(blue.orientation);
            //globalvy = blue.vel_x * sin(blue.orientation) - blue.vel_y * cos(blue.orientation);
            //使用之前场地计算的模块
            //globalvx = blue.vel_x * cos(blue.orientation) - blue.vel_y * sin(blue.orientation);
            //globalvy = blue.vel_x * sin(blue.orientation) + blue.vel_y * cos(blue.orientation);

            globalvx = blue.vel_x;
            globalvy = -blue.vel_y;
            //theta = 3.14159*(-1.0/3);
            theta = atan2(-blue.vel_y, blue.vel_x);
            endx = blue.x + PARAMS::OBSTACLE::REFRESH_TIME *globalvx;//*30* cos(theta);//;
            endy = blue.y + PARAMS::OBSTACLE::REFRESH_TIME *globalvy;//*30* sin(theta);//

            x[0] = blue.x - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4-theta);
            y[0] = blue.y + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4-theta);

            x[1] = blue.x - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4+theta);
            y[1] = blue.y - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4+theta);

            x[2] = blue.x + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4+theta);
            y[2] = blue.y + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4+theta);

            x[3] = blue.x + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4-theta);
            y[3] = blue.y - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4-theta);

            //qDebug() << "blue.id" << blue.robot_id << "V^2" << (pow(blue.vel_x,2)+pow(blue.vel_y,2));
            if(pow(blue.vel_x,2)+pow(blue.vel_y,2) <= 1000)
            {
                first = 0;
                second = 3;
                third = 1;
                forth = 2;
            }
            else
            {
                x[4] = endx - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4-theta);
                y[4] = endy + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4-theta);

                x[5] = endx - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4+theta);
                y[5] = endy - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4+theta);

                x[6] = endx + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4+theta);
                y[6] = endy + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4+theta);

                x[7] = endx + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4-theta);
                y[7] = endy - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4-theta);

                //找到最边上的四个点
                maxdistance = -1;
                for(int i = 0; i < 8; i++)
                {
                    for(int j = i+1; j < 8; j++)
                    {
                        mydis = pow(x[j]-x[i],2)+pow(y[j]-y[i],2);
    //                    qDebug() << "i" << i << "j" << j << "mydis" << mydis << "maxdis" << maxdistance;
                        if(mydis > maxdistance)
                        {
                            first = i;
                            second = j;
                            maxdistance = mydis;

                        }
                    }
                }
                maxdistance = -1;
                for(int i = 0; i < 8; i++)
                {
                    for(int j = i+1; j < 8; j++)
                    {
                        if(i == first && j == second) continue;
                        mydis = pow(x[j]-x[i],2)+pow(y[j]-y[i],2);
                        if(mydis > maxdistance)
                        {
                            third = i;
                            forth = j;
                            maxdistance = mydis;
                        }
                    }
                }
            }


            //绘制直线
            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_WHITE);
            line = msg->mutable_line();
            line->mutable_start()->set_x(x[first]);
            line->mutable_start()->set_y(y[first]);
            line->mutable_end()->set_x(x[third]);
            line->mutable_end()->set_y(y[third]);
            line->set_forward(false);
            line->set_back(false);

            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_WHITE);
            line = msg->mutable_line();
            line->mutable_start()->set_x(x[first]);
            line->mutable_start()->set_y(y[first]);
            line->mutable_end()->set_x(x[forth]);
            line->mutable_end()->set_y(y[forth]);
            line->set_forward(false);
            line->set_back(false);

            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_WHITE);
            line = msg->mutable_line();
            line->mutable_start()->set_x(x[second]);
            line->mutable_start()->set_y(y[second]);
            line->mutable_end()->set_x(x[third]);
            line->mutable_end()->set_y(y[third]);
            line->set_forward(false);
            line->set_back(false);

            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_WHITE);
            line = msg->mutable_line();
            line->mutable_start()->set_x(x[second]);
            line->mutable_start()->set_y(y[second]);
            line->mutable_end()->set_x(x[forth]);
            line->mutable_end()->set_y(y[forth]);
            line->set_forward(false);
            line->set_back(false);
        }
        // yellow robot
        if(MyDataManager::instance()->validYellowRobots[i] && !((PARAMS::IS_SIMULATION ? i==(PARAMS::our_id-1) : i==(PARAMS::our_id-1)) && !PARAMS::isBlue)){
            RobotInfo& yellow = MyDataManager::instance()->yellowRobots[i];
            //尤其注意，这里是路子豪写的，是按照正常情况下的X-Y坐标系来做的，对于仿真和实际的环境有点痴迷，详情请询问路子豪
            //globalvx = yellow.vel_x * cos(yellow.orientation) + yellow.vel_y * sin(yellow.orientation);
            //globalvy = yellow.vel_x * sin(yellow.orientation) - yellow.vel_y * cos(yellow.orientation);
//            theta = atan2(globalvy, globalvx);
            globalvx = yellow.vel_x;
            globalvy = -yellow.vel_y;
            //theta = 3.14159*(-1.0/3);
            theta = atan2(-yellow.vel_y, yellow.vel_x);
            endx = yellow.x + PARAMS::OBSTACLE::REFRESH_TIME *globalvx;//*30* cos(theta);//
            endy = yellow.y + PARAMS::OBSTACLE::REFRESH_TIME *globalvy;//*30* sin(theta);//


            x[0] = yellow.x - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4-theta);
            y[0] = yellow.y + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4-theta);

            x[1] = yellow.x - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4+theta);
            y[1] = yellow.y - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4+theta);

            x[2] = yellow.x + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4+theta);
            y[2] = yellow.y + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4+theta);

            x[3] = yellow.x + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4-theta);
            y[3] = yellow.y - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4-theta);

            if(pow(yellow.vel_x,2)+pow(yellow.vel_y,2) <= 1000)
            {
                first = 0;
                second = 3;
                third = 1;
                forth = 2;
            }
            else
            {
                x[4] = endx - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4-theta);
                y[4] = endy + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4-theta);

                x[5] = endx - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4+theta);
                y[5] = endy - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4+theta);

                x[6] = endx + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4+theta);
                y[6] = endy + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4+theta);

                x[7] = endx + sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*cos(PARAMS::MATH::PI/4-theta);
                y[7] = endy - sqrt(2)*PARAMS::OBSTACLE::INFLATION_RADIUS*sin(PARAMS::MATH::PI/4-theta);

                //找到最边上的四个点
                maxdistance = -1;
                for(int i = 0; i < 8; i++)
                {
                    for(int j = i+1; j < 8; j++)
                    {
                        //debug,把线全部画出来
    //                    msg = msgs.add_msgs();
    //                    msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
    //                    msg->set_color(ZSS::Protocol::Debug_Msg_Color_RED);
    //                    line = msg->mutable_line();
    //                    line->mutable_start()->set_x(x[i]);
    //                    line->mutable_start()->set_y(y[i]);
    //                    line->mutable_end()->set_x(x[j]);
    //                    line->mutable_end()->set_y(y[j]);
    //                    line->set_forward(false);
    //                    line->set_back(false);


                        mydis = pow(x[j]-x[i],2)+pow(y[j]-y[i],2);
    //                    qDebug() << "i" << i << "j" << j << "mydis" << mydis << "maxdis" << maxdistance;
                        if(mydis > maxdistance)
                        {
                            first = i;
                            second = j;
                            maxdistance = mydis;

                        }
                    }
                }
                maxdistance = -1;
                for(int i = 0; i < 8; i++)
                {
                    for(int j = i+1; j < 8; j++)
                    {
                        if(i == first && j == second) continue;
                        mydis = pow(x[j]-x[i],2)+pow(y[j]-y[i],2);
                        if(mydis > maxdistance)
                        {
                            third = i;
                            forth = j;
                            maxdistance = mydis;
                        }
                    }
                }
            }


//            qDebug() << x[0] << x[1] << x[2] << x[3] << x[4] << x[5] << x[6] << x[7];
//            qDebug() << y[0] << y[1] << y[2] << y[3] << y[4] << y[5] << y[6] << y[7];
//            qDebug() << x[first] << x[second] << x[third] << x[forth];
//            qDebug() << y[first] << y[second] << y[third] << y[forth];
//            qDebug() << first << second << third << forth;

            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_WHITE);
            line = msg->mutable_line();
            line->mutable_start()->set_x(x[first]);
            line->mutable_start()->set_y(y[first]);
            line->mutable_end()->set_x(x[third]);
            line->mutable_end()->set_y(y[third]);
            line->set_forward(false);
            line->set_back(false);

            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_WHITE);
            line = msg->mutable_line();
            line->mutable_start()->set_x(x[first]);
            line->mutable_start()->set_y(y[first]);
            line->mutable_end()->set_x(x[forth]);
            line->mutable_end()->set_y(y[forth]);
            line->set_forward(false);
            line->set_back(false);

            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_WHITE);
            line = msg->mutable_line();
            line->mutable_start()->set_x(x[second]);
            line->mutable_start()->set_y(y[second]);
            line->mutable_end()->set_x(x[third]);
            line->mutable_end()->set_y(y[third]);
            line->set_forward(false);
            line->set_back(false);

            msg = msgs.add_msgs();
            msg->set_type(ZSS::Protocol::Debug_Msg_Debug_Type_LINE);
            msg->set_color(ZSS::Protocol::Debug_Msg_Color_WHITE);
            line = msg->mutable_line();
            line->mutable_start()->set_x(x[second]);
            line->mutable_start()->set_y(y[second]);
            line->mutable_end()->set_x(x[forth]);
            line->mutable_end()->set_y(y[forth]);
            line->set_forward(false);
            line->set_back(false);
        }
    }
    //终尾

    int msgs_size = msgs.ByteSize();
    QByteArray msgs_data(msgs_size, 0);
    msgs.SerializeToArray(msgs_data.data(), msgs_data.size());
    sender->writeDatagram(msgs_data, msgs_size, PARAMS::visualAddress, PARAMS::visualPort);
}
