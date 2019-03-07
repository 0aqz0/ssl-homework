#ifndef UDPRECEIVER_H
#define UDPRECEIVER_H

#include <QtNetwork>

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

#endif // UDPRECEIVER_H
