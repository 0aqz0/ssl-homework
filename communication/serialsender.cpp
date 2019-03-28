#include "serialsender.h"
#include <QDebug>
#include "utils/params.h"

serialSender::serialSender()
    : startPacket1(PARAMS::TRANSMIT_PACKET_SIZE,0)
    , startPacket2(PARAMS::TRANSMIT_PACKET_SIZE,0)
    , transmitPacket(PARAMS::TRANSMIT_PACKET_SIZE,0)
{
    startPacket1[0] = 0xff;
    startPacket1[1] = 0xb0;
    startPacket1[2] = 0x01;
    startPacket1[3] = 0x02;
    startPacket1[4] = 0x03;
    startPacket1[24] = 0x31;

    startPacket2[0] = 0xff;
    startPacket2[1] = 0xb0;
    startPacket2[2] = 0x04;
    startPacket2[3] = 0x05;
    startPacket2[4] = 0x06;
    startPacket2[5] = 0x10 + PARAMS::frequency;
    startPacket2[24] = 0x85;
}

void serialSender::openSerialPort()
{
    serial.setPortName(PARAMS::serialPort);
    serial.setBaudRate(QSerialPort::Baud115200);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);

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
    serial.write(startPacket1.data(), PARAMS::TRANSMIT_PACKET_SIZE);
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
    serial.write(startPacket2.data(), PARAMS::TRANSMIT_PACKET_SIZE);
    serial.flush();
}

void serialSender::encode(int robotID, int velX, int velY, int velR, bool ctrl, bool shootMode, int ctrlPowerLevel)
{
    for (int i=0; i<PARAMS::TRANSMIT_PACKET_SIZE; i++){
        transmitPacket[i] = 0x00;
    }
//    transmitPacket[0] = 0xff;
//    //RobotID
//    if(robot_id >= 8)
//        transmitPacket[1] = 0x01 << (robot_id - 8);
//    else
//        transmitPacket[1] = 0x00;
//    transmitPacket[2] =  0x01 << robot_id;
//    // misc
//    transmitPacket[3] = transmitPacket[3] | (ctrl << 7);
//    //shoot or chip
//    transmitPacket[3] = transmitPacket[3] | (shootMode << 6);
//    //power level
//    transmitPacket[3] = transmitPacket[3] | (ctrlPowerLevel << 4);
//    // other
//    transmitPacket[3] = transmitPacket[3] & 0xf0;
//    transmitPacket[3] = transmitPacket[3] | 0x01;

//    // velx
//    if(vel_x < 0) transmitPacket[4] = transmitPacket[4] | (0x20);
//    transmitPacket[4] = transmitPacket[4] | ((abs(vel_x) & 0x1f0) >> 4);
////    qDebug() << "ddq debuging: " << (abs(velX) & 0x1f0);
//    transmitPacket[5] = transmitPacket[5] | ((abs(vel_x) & 0x0f) << 4);
//    // vely
//    if(vel_y < 0) transmitPacket[5] = transmitPacket[5] | (0x08);
//    transmitPacket[5] = transmitPacket[5] | ((abs(vel_y) & 0x1c0) >> 6);
//    transmitPacket[6] = transmitPacket[6] | ((abs(vel_y) & 0x3f) << 2);
//    // w
//    if(vel_w < 0) transmitPacket[6] = transmitPacket[6] | 0x02;
//    transmitPacket[6] = transmitPacket[6] | ((abs(vel_w) & 0x100) >> 8);
//    transmitPacket[7] = transmitPacket[7] | (abs(vel_w) & 0x0ff);

//    // shoot power
//    transmitPacket[8] = 0x00;

//    transmitPacket[21] = 0x07;
    transmitPacket[0] = 0x48;
    //RobotID
    transmitPacket[1] = (robotID) & 0x0f;
//    transmitPacket[0] = transmitPacket[0] | 0x08;
    //Robot1 Config
    //shoot or chip
    transmitPacket[1] = transmitPacket[1] | (shootMode << 6 );
    //power level
    transmitPacket[1] = transmitPacket[1] | (ctrl ? (ctrlPowerLevel << 4):0);
    //速度的低位
    transmitPacket[2] = ((velX >= 0)?0:0x80) | (abs(velX) & 0x7f);
    transmitPacket[3] = ((velY >= 0)?0:0x80) | (abs(velY) & 0x7f);
    transmitPacket[4] = ((velR >= 0)?0:0x80) | (abs(velR) & 0x7f);
    //Don't understand !
    if(transmitPacket[2] == char(0xff)) transmitPacket[4] = 0xfe;
    if(transmitPacket[3] == char(0xff)) transmitPacket[5] = 0xfe;
    if(transmitPacket[4] == char(0xff)) transmitPacket[6] = 0xfe;
    //clear Byte[17-24]
    transmitPacket[17] = transmitPacket[18] = transmitPacket[19] = transmitPacket[20] = transmitPacket[21] = transmitPacket[22] = transmitPacket[23] = transmitPacket[24] = 0;
    //速度的高位
    transmitPacket[17] = ((abs(velX) & 0x180) >> 1) | ((abs(velY) & 0x180) >> 3) | ((abs(velR) & 0x780) >> 7);
    //shoot power
    transmitPacket[21] = 0 & 0x7f;

}

void serialSender::sendToReal(int robot_id, int vel_x, int vel_y, int vel_w)
{
    encode(robot_id-1, vel_x, vel_y, vel_w);
//    qDebug() << "0x" << transmitPacket.toHex();
    serial.write(transmitPacket.data(), PARAMS::TRANSMIT_PACKET_SIZE);
//    serial.flush();
    serial.waitForBytesWritten(-1);
}
