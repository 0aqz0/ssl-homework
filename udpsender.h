#ifndef UDPSENDER_H
#define UDPSENDER_H

#include <QObject>
#include <QtNetwork>
#include "utils/singleton.hpp"

class UDPSender : public QObject
{
    Q_OBJECT
public:
    explicit UDPSender(QObject *parent = nullptr);
    ~UDPSender();
    void sendToSim(int robot_id=0, double vel_x=0, double vel_y=0, double vel_w=0);
private:
    QUdpSocket* sender;
};
typedef Singleton<UDPSender> CommandSender;

#endif // UDPSENDER_H
