#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#include <QtNetwork>

class UDPReceiver
{
public:
    static UDPReceiver* instance();
    void readDatagrams();
private:
    UDPReceiver();
    ~UDPReceiver();
    static UDPReceiver* _instance;
    QUdpSocket *receiver;
};


#endif // UDPRECEIVER_H
