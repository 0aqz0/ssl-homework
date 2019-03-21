#ifndef SERIALSENDER_H
#define SERIALSENDER_H

#include <QSerialPort>
#include "utils/singleton.hpp"

class serialSender
{
public:
    serialSender();
    void openSerialPort();
    void closeSerialPort();
    void sendStartPacket();
    void encode(int robot_id=0, int vel_x=0, int vel_y=0, int vel_w=0, bool ctrl=false, bool shootMode=false, int ctrlPowerLevel=0);
    void sendToReal(int robot_id=0, int vel_x=0, int vel_y=0, int vel_w=0);
private:
    QSerialPort serial;
    QByteArray startPacket1;
    QByteArray startPacket2;
    QByteArray transmitPacket;
};
typedef Singleton<serialSender> RealCommandSender;

#endif // SERIALSENDER_H
