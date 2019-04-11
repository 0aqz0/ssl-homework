#ifndef VISUALIZATIONMODULE_H
#define VISUALIZATIONMODULE_H

#include <vector>
#include <QObject>
#include <QtNetwork>
#include "utils/singleton.hpp"
#include "utils/mymath.h"
#include "proto/zss_debug.pb.h"
#include "proto/grSim_Packet.pb.h"
#include <deque>

class visualizationmodule : public QObject
{
    Q_OBJECT
public:
    explicit visualizationmodule(QObject *parent = nullptr);
    ~visualizationmodule();
    void drawTree(std::vector<Node> &someNodes);
    void drawPoints(std::vector<MyPoint> &somePoints);
    void drawLines(std::vector<MyPoint> &somePoints);
    void drawPoint(MyPoint point);
    void drawAll(std::deque<MyPoint> goals);
private:
    QUdpSocket* sender;
signals:

public slots:
};

typedef Singleton<visualizationmodule> VisualModule;
#endif // VISUALIZATIONMODULE_H
