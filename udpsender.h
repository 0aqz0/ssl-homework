#ifndef UDPSENDER_H
#define UDPSENDER_H

#include <QObject>
#include <QtNetwork>

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

#endif // UDPSENDER_H
