#include <QCoreApplication>
#include "udpreceiver.h"
#include "udpsender.h"
#include "serialsender.h"
#include <thread>

void loop()
{
//    RealCommandSender::instance()->openSerialPort();
//    RealCommandSender::instance()->sendStartPacket();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        qDebug() << "in the thread!";
//        RealCommandSender::instance()->sendToReal(0, 100, 0, 0);
    }
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    VisionReceiver::instance();

    std::thread* _thread = new std::thread([ = ] {loop();});

    return a.exec();
}
