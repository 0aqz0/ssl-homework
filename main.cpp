#include <QCoreApplication>
#include "udpreceiver.h"
#include "udpsender.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    UDPReceiver m_receiver;
    UDPSender m_sender;
    while(true)
        m_sender.sendCommand();

    return a.exec();
}
