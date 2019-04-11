#include "rrt.h"
#include "utils/params.h"
#include <QRandomGenerator>
#include "algorithm/obstacles.h"

void RRT::plan(double start_x, double start_y, double end_x, double end_y)
{
    // exploration
    NodeList.clear();
    NodeList.push_back(Node(start_x, start_y));

    for(int iter=0; iter<PARAMS::RRT::ITERATIONS; iter++) {
        // debug info
        if(iter > PARAMS::RRT::ITERATIONS - 10 && PARAMS::DEBUG::RRTDebug)
            qDebug() << "planning fials!!!";
        // random sampling
        Node randNode = randomSample(PARAMS::RRT::EPSILON, end_x, end_y);
        // Find the nearest node
        int nearestNode = findNearestNode(randNode.x, randNode.y);
        // expand the tree
        double theta = atan2(randNode.y - NodeList[nearestNode].y, randNode.x - NodeList[nearestNode].x);
        int newNode_x = NodeList[nearestNode].x + PARAMS::RRT::STEP_SIZE*cos(theta);
        int newNode_y = NodeList[nearestNode].y + PARAMS::RRT::STEP_SIZE*sin(theta);

        if(PARAMS::DEBUG::RRTDebug)
            qDebug() << "newNode: " << newNode_x << newNode_y << nearestNode;

        // outside the map
        if (newNode_x <= -PARAMS::FIELD::LENGTH/2 || newNode_y <= -PARAMS::FIELD::WIDTH/2 || newNode_x >= PARAMS::FIELD::LENGTH/2 || newNode_y >= PARAMS::FIELD::WIDTH/2)
            continue;
        // in the closed list
        if (inNodeList(newNode_x, newNode_y))
            continue;
        // obstacles
        if (ObstaclesInfo::instance()->hasObstacle(newNode_x, newNode_y, PARAMS::OBSTACLE::OBSTACLETYPE))
        {
//            qDebug() << "obstacle!!!";
            continue;

        }


        NodeList.push_back(Node(newNode_x, newNode_y, nearestNode));
        if (sqrt(pow(newNode_x-end_x,2)+pow(newNode_y-end_y,2)) < PARAMS::RRT::STEP_SIZE){
            if(PARAMS::DEBUG::RRTDebug)
                qDebug() <<"final";
            // generate the final path
            std::vector<MyPoint> tempPath;
            tempPath.push_back(MyPoint(end_x, end_y));
            tempPath.push_back(MyPoint(newNode_x, newNode_y));
            int currentNode = NodeList.back().parent;
            while(currentNode>=0){
                tempPath.push_back(MyPoint(NodeList[currentNode].x, NodeList[currentNode].y));
                currentNode = NodeList[currentNode].parent;
                if(PARAMS::DEBUG::RRTDebug)
                    qDebug() <<"generate temp path";
            }
            if(PARAMS::DEBUG::RRTDebug)
                qDebug() <<"finish temp path";
            // invert the path
            finalPath.clear();
            while(tempPath.size()){
                finalPath.push_back(tempPath.back());
                tempPath.pop_back();
                if(PARAMS::DEBUG::RRTDebug)
                    qDebug() <<"generate final path";
            }
            if(PARAMS::DEBUG::RRTDebug)
                qDebug() <<"finish final path";
            pathSmooth();
            if(PARAMS::DEBUG::RRTDebug)
                qDebug() <<"finish smooth path";
            break;
        }
    }
}

int RRT::findNearestNode(int x, int y)
{
    double max_distance = 1e10;
    int nearest_index = -1;
    for(int i = 0; i < NodeList.size(); i++){
        double distance = sqrt(pow(NodeList[i].x-x,2) + pow(NodeList[i].y-y,2));
        if(distance < max_distance){
            max_distance = distance;
            nearest_index = i;
        }
    }
    return nearest_index;
}

Node RRT::randomSample(double epsilon, int end_x, int end_y)
{
    int rand_x, rand_y;

    if(QRandomGenerator::global()->generate()%100 >= epsilon*100){
        rand_x = QRandomGenerator::global()->generate()%PARAMS::FIELD::LENGTH - PARAMS::FIELD::LENGTH/2;
        rand_y = QRandomGenerator::global()->generate()%PARAMS::FIELD::WIDTH - PARAMS::FIELD::WIDTH/2;
    }else{
        rand_x = end_x;
        rand_y = end_y;
    }
    return Node(rand_x, rand_y);
}

bool RRT::inNodeList(int x, int y)
{
    for(int i=0; i<NodeList.size(); i++){
        if(NodeList[i].x == x && NodeList[i].y == y)
            return true;
    }
    return false;
}

void RRT::pathSmooth()
{
    smoothPath.clear();
    smoothPath.push_back(finalPath[0]);
    int nextPoint = 1;
    while(nextPoint < finalPath.size()){
        while(nextPoint < finalPath.size()-1 && !ObstaclesInfo::instance()->hasObstacle(smoothPath.back().x(), smoothPath.back().y(), finalPath[nextPoint+1].x(), finalPath[nextPoint+1].y(), PARAMS::OBSTACLE::OBSTACLETYPE)){
            nextPoint++;
            if(PARAMS::DEBUG::RRTDebug)
                qDebug() << "jump!!!";
        }
        smoothPath.push_back(finalPath[nextPoint]);
        nextPoint++;
    }
}
