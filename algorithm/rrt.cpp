#include "rrt.h"

void RRT::plan(unsigned char *costs, float *potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> > &path)
{
//    int ns = xs_ * ys_;
//    long POT_HIGH = 1e10;

//    // exploration
//    NodeList.clear();
//    NodeList.push_back(Node(start_x, start_y));

//    std::fill(potential, potential + ns, POT_HIGH);
//    potential[getIndex(start_x, start_y)] = 0;

//    int cycle = 0;
//    int cycles = 1e4;
//    int stepsize = 10;

//    int rand_x, rand_y;
//    std::default_random_engine generator;
//    std::uniform_int_distribution<int> distribution(1800,2200);
//    while (cycle < cycles) {
//        // random sampling
//        if(rand()%100 >= 5){
//            rand_x = distribution(generator);
//            rand_y = distribution(generator);
//        }else{
//            rand_x = end_x;
//            rand_y = end_y;
//        }

//        // Find the nearest node
//        int nearestNode = findNearestNode(rand_x, rand_y);

//        // // expand the tree
//        float theta = atan2(rand_y - NodeList[nearestNode].y, rand_x - NodeList[nearestNode].x)*180/3.14;

//        int newNode_x = NodeList[nearestNode].x + stepsize*cos(theta);
//        int newNode_y = NodeList[nearestNode].y + stepsize*sin(theta);

//        // outside the map
//        if (newNode_x < 0 || newNode_y < 0 || newNode_x >= xs_ || newNode_y >= ys_)
//            continue;
//        // in the closed list
//        if (potential[getIndex(newNode_x, newNode_y)] < POT_HIGH)
//            continue;
//        // obstacles
//        if (costs[getIndex(newNode_x, newNode_y)] > 0)
//            continue;
//        if(costs[getIndex(newNode_x, newNode_y)]>=lethal_cost_)
//            continue;

//        potential[getIndex(newNode_x, newNode_y)] = potential[getIndex(NodeList[nearestNode].x, NodeList[nearestNode].y)] + 1;
//        NodeList.push_back(Node(newNode_x, newNode_y, nearestNode));

//        if (sqrt(pow(newNode_x-end_x,2)+pow(newNode_y-end_y,2)) < stepsize){
//            break;
//        }
//        cycle++;
//    }

//    // generate the final path
//    std::pair<float, float> current;
//    current.first = end_x;
//    current.second = end_y;
//    path.push_back(current);
//    int currentNode = NodeList.size()-1;
//    while (currentNode>=0) {
//        current.first = NodeList[currentNode].x;
//        current.second = NodeList[currentNode].y;
//        path.push_back(current);
//        currentNode = NodeList[currentNode].parent;
//    }
}

int RRT::findNearestNode(int x, int y)
{
    float max_distance = 1e10;
    int nearest_index = -1;
    for(int i = 0; i < NodeList.size(); i++){
        float distance = sqrt(pow(NodeList[i].x-x,2) + pow(NodeList[i].y-y,2));
        if(distance < max_distance){
            max_distance = distance;
            nearest_index = i;
        }
    }
    return nearest_index;
}

void RRT::pathSmooth()
{

}
