#include "datamanager.h"
namespace {
    int CAR_NUM = 16;
}

DataManager::DataManager()
{
    reset();
}

void DataManager::reset()
{
    for (int i=0; i<CAR_NUM; i++){
        validBlueRobots[i] = false;
        validYellowRobots[i] = false;
    }
}
