#ifndef PARAMS_H
#define PARAMS_H
#include <QString>
#include <QHostAddress>

namespace PARAMS {
    const bool IS_SIMULATION = true;
    const double ACCEPT_RADIUS = 10.0;

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

    // path planning
    const double ANGLE_THRESHOLD = 0.1;
    const double FARWARD_ANGLE_THRESHOLD = 0.5;
    const double ROTATE_COFF = 1.0;

    // math constant
    namespace MATH {
        const double PI = 3.14159265359;
    }
}

#endif // PARAMS_H
