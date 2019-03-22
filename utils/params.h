#ifndef PARAMS_H
#define PARAMS_H
#include <QString>
#include <QHostAddress>

namespace PARAMS {
    const bool IS_SIMULATION = true;
    const double ACCEPT_RADIUS = 5.0;

    // data manager
    const int ROBOT_NUM = 16;

    // our car
    const int our_id = 0;
    const bool isBlue = true;

    // serial sender
    const QString serialPort = "COM7";
    const int frequency = 0;
    const int TRANSMIT_PACKET_SIZE = 25;

    // UDP receiver
    const int visionPort = 23333;

    // UDP sender
    const int simPort = 20011;
    const QHostAddress simAddress = QHostAddress("127.0.0.1");
}

#endif // PARAMS_H
