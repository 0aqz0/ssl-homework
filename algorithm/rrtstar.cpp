#include "rrtstar.h"
#include "utils/params.h"
#include <QRandomGenerator>
#include "algorithm/obstacles.h"

void RRTStar::plan(double start_x, double start_y, double end_x, double end_y)
{
    // exploration
    NodeList.clear();
    NodeList.push_back(Node(start_x, start_y));

    for(int iter=0; iter<PARAMS::RRTStar::ITERATIONS; iter++) {
        // random sampling
        Node randNode = randomSample(PARAMS::RRTStar::EPSILON, end_x, end_y);
        // Find the nearest node
        int nearestNode = findNearestNode(randNode.x, randNode.y);
        // expand the tree
        double theta = atan2(randNode.y - NodeList[nearestNode].y, randNode.x - NodeList[nearestNode].x);
        int newNode_x = NodeList[nearestNode].x + PARAMS::RRTStar::STEP_SIZE*cos(theta);
        int newNode_y = NodeList[nearestNode].y + PARAMS::RRTStar::STEP_SIZE*sin(theta);
        double newNode_cost = NodeList[nearestNode].cost + sqrt(pow(newNode_x - NodeList[nearestNode].x, 2) + pow(newNode_y - NodeList[nearestNode].y, 2));
        int newNode_parent = nearestNode;

        // outside the map
        if (newNode_x <= -PARAMS::FIELD::LENGTH/2 || newNode_y <= -PARAMS::FIELD::WIDTH/2 || newNode_x >= PARAMS::FIELD::LENGTH/2 || newNode_y >= PARAMS::FIELD::WIDTH/2)
            continue;
        // in the closed list
        if (inNodeList(newNode_x, newNode_y))
            continue;
        // obstacles
        if (ObstaclesInfo::instance()->hasObstacle(newNode_x, newNode_y, PARAMS::OBSTACLE::OBSTACLETYPE))
            continue;

        // choose best parent
        double radius = PARAMS::RRTStar::GAMMA*sqrt(log(NodeList.size())/NodeList.size());
        int bestParent = chooseBestParent(newNode_x, newNode_y, radius, newNode_cost);
        if(bestParent>=0){
            newNode_parent = bestParent;
            newNode_cost = NodeList[newNode_parent].cost + sqrt(pow(NodeList[newNode_parent].y - newNode_y, 2)+pow(NodeList[newNode_parent].x - newNode_x, 2));
        }
        Node newNode = Node(newNode_x, newNode_y, newNode_parent, newNode_cost);
        NodeList.push_back(newNode);

        // rewire
        rewire(newNode, radius);

        if (sqrt(pow(newNode_x-end_x, 2) + pow(newNode_y-end_y, 2)) < PARAMS::RRTStar::STEP_SIZE)
            break;
    }
    // generate the final path
    std::vector<MyPoint> tempPath;
    tempPath.clear();
    tempPath.push_back(MyPoint(end_x, end_y));
    int currentNode = NodeList.back().parent;
    while(currentNode>=0){
        tempPath.push_back(MyPoint(NodeList[currentNode].x, NodeList[currentNode].y));
        currentNode = NodeList[currentNode].parent;
    }
    // invert the path
    finalPath.clear();
    while(tempPath.size()){
        finalPath.push_back(tempPath.back());
        tempPath.pop_back();
    }
    pathSmooth();
//    for(int i=0; i<finalPath.size();i++){
//        qDebug() << finalPath[i].x() << finalPath[i].y();
//    }
}

int RRTStar::chooseBestParent(int x, int y, double radius, double old_cost)
{
    // find near nodes
    std::vector<Node> nearNodes;
    std::vector<int> nearNodesIndex;
    nearNodes.clear();
    nearNodesIndex.clear();
    int bestParent = -1;
    for(int i=0; i<NodeList.size(); i++){
        if(pow(NodeList[i].x - x, 2)+pow(NodeList[i].y - y, 2) < pow(radius, 2)){
            nearNodes.push_back(NodeList[i]);
            nearNodesIndex.push_back(i);
        }
    }
    // choose best parent
    if(nearNodes.size()){
        for (int i=0; i<nearNodes.size(); i++)
        {
            double new_cost = nearNodes[i].cost + sqrt(pow(nearNodes[i].y - y, 2) + pow(nearNodes[i].x - x, 2));
            // check collision
            if(ObstaclesInfo::instance()->hasObstacle(nearNodes[i].x, nearNodes[i].y, x, y, PARAMS::OBSTACLE::OBSTACLETYPE))
                continue;

            if(new_cost < old_cost){
                bestParent = nearNodesIndex[i];
                old_cost = new_cost;
            }
        }
    }
    return bestParent;
}

void RRTStar::rewire(Node newNode, double radius)
{
    // find near nodes
    std::vector<Node> nearNodes;
    nearNodes.clear();
    for(int i=0; i<NodeList.size(); i++){
        if(pow(NodeList[i].x - newNode.x, 2)+pow(NodeList[i].y - newNode.y, 2) < pow(radius, 2)){
            nearNodes.push_back(NodeList[i]);
        }
    }
    // rewire
    for(int i=0; i<nearNodes.size(); i++){
        double new_cost = newNode.cost + sqrt(pow(newNode.y - nearNodes[i].y, 2) + pow(newNode.x - nearNodes[i].x, 2));
        // check collision
//        if(ObstaclesInfo::instance()->hasObstacle(nearNodes[i].x, nearNodes[i].y, newNode.x, newNode.y, PARAMS::OBSTACLE::OBSTACLETYPE))
//            continue;
        if(new_cost < nearNodes[i].cost){
            nearNodes[i].cost = new_cost;
            nearNodes[i].parent = NodeList.size()-1;
        }
    }
}

int RRTStar::findNearestNode(int x, int y)
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

Node RRTStar::randomSample(double epsilon, int end_x, int end_y)
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

bool RRTStar::inNodeList(int x, int y)
{
    for(int i=0; i<NodeList.size(); i++){
        if(NodeList[i].x == x && NodeList[i].y == y)
            return true;
    }
    return false;
}

void RRTStar::pathSmooth()
{
    smoothPath.clear();
    smoothPath.push_back(finalPath[0]);
    int nextPoint = 1;
    while(nextPoint < finalPath.size()){
        while(!ObstaclesInfo::instance()->hasObstacle(smoothPath.back().x(), smoothPath.back().y(), finalPath[nextPoint+1].x(), finalPath[nextPoint+1].y(), PARAMS::OBSTACLE::OBSTACLETYPE) && nextPoint < finalPath.size()-1)
            nextPoint++;
        smoothPath.push_back(finalPath[nextPoint]);
        nextPoint++;
    }
}


