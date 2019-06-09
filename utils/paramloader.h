#ifndef PARAMLOADER_H
#define PARAMLOADER_H
#include <QString>
#include <QFile>
#include <QDebug>
#include <QDir>
#include "utils/singleton.hpp"

class paramLoader
{
public:
    paramLoader();
    QString COM;
    double forwardSpeed;
    double rotateSpeed;
    double forwardRotateCoff;
private:
    void loadParam();

};
typedef Singleton<paramLoader> ParamManager;

#endif // PARAMLOADER_H
