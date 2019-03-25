#ifndef DEBUG_H
#define DEBUG_H


#include <vector>
#include <QObject>
#include <QtNetwork>
#include "utils/singleton.hpp"
#include "utils/mymath.h"

class Debug : public QObject
{
    Q_OBJECT
public:
    explicit Debug(QObject *parent = nullptr);
    ~Debug();
    void sendToSim(std::vector<Node> &someNodes);
    void sendToSim(std::vector<MyPoint> &somepoints);
private:
    QUdpSocket* sender;
};

typedef Singleton<Debug> DebugMsgSender;
#endif // DEBUG_H
