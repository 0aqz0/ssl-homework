#include <QCoreApplication>
#include "udpreceiver.h"
#include "udpsender.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    VisionReceiver::instance();
    while(true){
        CommandSender::instance()->sendCommand();
    }

    return a.exec();
}
