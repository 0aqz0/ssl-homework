#include "paramloader.h"

namespace {
QString filename = "params.txt";
}

paramLoader::paramLoader()
{
    loadParam();
}

void paramLoader::loadParam()
{
    QString file = QDir::currentPath() + "/" + filename;
    QFile inputFile(file);
    if(inputFile.open(QIODevice::ReadOnly)){
        qDebug() << "OPEN PARAM FILE SUCCESS";
        QTextStream in(&inputFile);
        COM = in.readLine();
        forwardSpeed = in.readLine().toDouble();
        rotateSpeed = in.readLine().toDouble();
        forwardRotateCoff = in.readLine().toDouble();
        inputFile.close();
    }
    else
        qDebug() << "OPEN PARAM FILE ERROR";
}
