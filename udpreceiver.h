#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#include <QtNetwork>
#include "utils/singleton.hpp"

class UDPReceiver : public QObject
{
    Q_OBJECT
public:
    UDPReceiver(QObject* parent = nullptr);
    ~UDPReceiver();
private:
    QUdpSocket *receiver;
private slots:
    void readDatagrams();
};
typedef Singleton<UDPReceiver> VisionReceiver;


#endif // UDPRECEIVER_H
