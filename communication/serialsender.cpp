#include "serialsender.h"
#include <QDebug>
namespace {
    QString portName = "COM7";
    const int TRANSMIT_PACKET_SIZE = 25;
    int frequency = 0;
}

serialSender::serialSender()
    : startPacket1(TRANSMIT_PACKET_SIZE,0)
    , startPacket2(TRANSMIT_PACKET_SIZE,0)
    , transmitPacket(TRANSMIT_PACKET_SIZE,0)
{
    startPacket1[0] = 0xff;
    startPacket1[1] = 0xb0;
    startPacket1[2] = 0x01;
    startPacket1[3] = 0x02;
    startPacket1[4] = 0x03;
    startPacket1[24] = 0x31;
//    startPacket1[TRANSMIT_PACKET_SIZE - 1] = CCrc8::calc((unsigned char*)(startPacket1.data()), TRANSMIT_PACKET_SIZE - 1);

    startPacket2[0] = 0xff;
    startPacket2[1] = 0xb0;
    startPacket2[2] = 0x04;
    startPacket2[3] = 0x05;
    startPacket2[4] = 0x06;
    startPacket2[5] = 0x10 + frequency;
    startPacket2[24] = 0x85;
//    startPacket2[TRANSMIT_PACKET_SIZE - 1] = CCrc8::calc((unsigned char*)(startPacket2.data()), TRANSMIT_PACKET_SIZE - 1);
}

void serialSender::openSerialPort()
{
    serial.setPortName(portName);
    serial.setBaudRate(QSerialPort::Baud115200);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::TwoStop);

    if (serial.open(QIODevice::ReadWrite))
        qDebug() << "Serial Port Connected!!!";
    else
        qDebug() << "Serial Port connection failed!!!";
}

void serialSender::closeSerialPort()
{
    if (serial.isOpen()){
        serial.close();
        qDebug() << "Serial Port Disconnected!!!";
    }
}

void serialSender::sendStartPacket()
{
    serial.write(startPacket1.data(), TRANSMIT_PACKET_SIZE);
    serial.flush();
    if (serial.waitForBytesWritten(2000)){
        if (serial.waitForReadyRead(2000)){
            QByteArray respenseData = serial.readAll();
            while (serial.waitForReadyRead(10))
                respenseData += serial.readAll();
        }
    }
    else
        qDebug() << "start packet time out!!!";
    qDebug() << "0x" << startPacket1.toHex();
    qDebug() << "0x" << startPacket2.toHex();
    serial.write(startPacket2.data(), TRANSMIT_PACKET_SIZE);
    serial.flush();
}

void serialSender::encode(int robot_id, int vel_x, int vel_y, int vel_w, bool ctrl, bool shootMode, int ctrlPowerLevel)
{
    for (int i=0; i<TRANSMIT_PACKET_SIZE; i++){
        transmitPacket[i] = 0x00;
    }
    transmitPacket[0] = 0xff;
    //RobotID
    if(robot_id >= 8)
        transmitPacket[1] = 0x01 << (robot_id - 8);
    else
        transmitPacket[1] = 0x00;
    transmitPacket[2] =  0x01 << robot_id;
    // misc
    transmitPacket[3] = transmitPacket[3] | (ctrl << 7);
    //shoot or chip
    transmitPacket[3] = transmitPacket[3] | (shootMode << 6);
    //power level
    transmitPacket[3] = transmitPacket[3] | (ctrlPowerLevel << 4);
    // other
    transmitPacket[3] = transmitPacket[3] & 0xf0;
    transmitPacket[3] = transmitPacket[3] | 0x01;

    // velx
    if(vel_x < 0) transmitPacket[4] = transmitPacket[4] | (0x20);
    transmitPacket[4] = transmitPacket[4] | ((abs(vel_x) & 0x1f0) >> 4);
//    qDebug() << "ddq debuging: " << (abs(velX) & 0x1f0);
    transmitPacket[5] = transmitPacket[5] | ((abs(vel_x) & 0x0f) << 4);
    // vely
    if(vel_y < 0) transmitPacket[5] = transmitPacket[5] | (0x08);
    transmitPacket[5] = transmitPacket[5] | ((abs(vel_y) & 0x1c0) >> 6);
    transmitPacket[6] = transmitPacket[6] | ((abs(vel_y) & 0x3f) << 2);
    // w
    if(vel_w < 0) transmitPacket[6] = transmitPacket[6] | 0x02;
    transmitPacket[6] = transmitPacket[6] | ((abs(vel_w) & 0x100) >> 8);
    transmitPacket[7] = transmitPacket[7] | (abs(vel_w) & 0x0ff);

    // shoot power
    transmitPacket[8] = 0x00;

    transmitPacket[21] = 0x07;
}

void serialSender::sendToReal(int robot_id, int vel_x, int vel_y, int vel_w)
{
    encode(robot_id, vel_x, vel_y, vel_w);
    qDebug() << "0x" << transmitPacket.toHex();
    serial.write(transmitPacket.data(), TRANSMIT_PACKET_SIZE);
    serial.flush();
}
