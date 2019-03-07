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
    void sendCommand();
private:
    QUdpSocket* sender;
};
typedef Singleton<UDPSender> CommandSender;

#endif // UDPSENDER_H
