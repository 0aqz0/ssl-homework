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

    // field
    namespace FIELD {
        const int LENGTH = 1200;
        const int WIDTH = 900;
    }

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
    const double FORWARD_ANGLE_THRESHOLD = 0.3;
    const double ROTATE_COFF = 4.0;
    const double FORWARD_SPEED = 3.0;
    const double FORWARD_ROTATE_COFF = 3.0;

    //visualization module
    const QHostAddress visualAddress = QHostAddress("127.0.0.1");
    const int visualPort = 20001;

    // RRT
    namespace RRT {
        const int ITERATIONS = 1e4;
        const int STEP_SIZE = 10;
        const double EPSILON = 0.05;
    }

    // RRTStar
    namespace RRTStar {
        const int ITERATIONS = 1e4;
        const int STEP_SIZE = 5;
        const double EPSILON = 0.05;
        const double GAMMA = 200.0;
    }

    // math constant
    namespace MATH {
        const double PI = 3.14159265359;
    }
}

#endif // PARAMS_H
