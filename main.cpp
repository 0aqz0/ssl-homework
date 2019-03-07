#include <QCoreApplication>
#include "udpreceiver.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    UDPReceiver m_receiver;

    return a.exec();
}
