#ifndef PARAMS_H
#define PARAMS_H
#include <QString>
#include <QHostAddress>

namespace PARAMS {
    const bool IS_SIMULATION = false;
    const double ACCEPT_RADIUS = 20.0;

    // data manager
    const int ROBOT_NUM = 12;

    // our car
    const int our_id = 3;  //从1开始
    const bool isBlue =true;

    // field
    namespace FIELD {
        const int LENGTH = 580;
        const int WIDTH = 380;
    }

    // serial sender
    const QString serialPort = "COM17";
    const int frequency = 0;
    const int TRANSMIT_PACKET_SIZE = 25;

    // UDP receiver
    const int visionPort = 23333;

    // UDP sender
    const int simPort = 20011;
    const QHostAddress simAddress = QHostAddress("127.0.0.1");

    // path planning
    const double ANGLE_THRESHOLD = 0.1;
    const double FORWARD_ANGLE_THRESHOLD = 0.5;
    const double ROTATE_COFF = 4;
    const double FORWARD_SPEED = 2;//原本是5
    const double FORWARD_ACC = 5;//原本是7.5
    const double FORWARD_ROTATE_COFF = 3.0;

    //visualization module
    const QHostAddress visualAddress = QHostAddress("127.0.0.1");
    const int visualPort = 20001;

    // RRT
    namespace RRT {
        const int ITERATIONS = 1e3;
        const int STEP_SIZE = 5;
        const double EPSILON = 0.1;
    }

    // RRTStar
    namespace RRTStar {
        const int ITERATIONS = 1e4;
        const int STEP_SIZE = 2;
        const double EPSILON = 0.1;
        const double GAMMA = 30.0;
    }

    // math constant
    namespace MATH {
        const double PI = 3.14159265359;
    }

    // FPS
    const int FRAME = 65;

    // obstacle module
    namespace OBSTACLE {
        //1 is CIRCLE , 2 is CAPSULE
        const int OBSTACLETYPE = 2;
        const double INFLATION_RADIUS = 20.0;//CAPSULE
        const double INFLATION_RADIUS_SQUARE = 900;//CIRCLE
        const double STEP_SIZE = 10;
        const double REFRESH_TIME = 0.02;
    }

    namespace DEBUG {
        // switch for artifical potential message
        const bool kAPDebugMessage  = false;
        const bool kMainDebug       = false;
        const bool kShowPathPlanner = false;
        const bool kgoToPosition2d  = false;
        const bool kUdpReceiver     = false;

        const bool RRTDebug = false;
        const bool pathPlannerDebug = false;
    }
}

#endif // PARAMS_H
