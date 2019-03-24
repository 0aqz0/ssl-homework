#include "rrtstar.h"

void RRTStar::plan(unsigned char *costs, float *potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> > &path)
{
//    int ns = xs_ * ys_;
//    long POT_HIGH = 1e10;

    // exploration
    NodeList.clear();
    NodeList.push_back(Node(start_x, start_y));

//    std::fill(potential, potential + ns, POT_HIGH);
//    potential[getIndex(start_x, start_y)] = 0;

    int cycle = 0;
    int cycles = 1e4;
    int stepsize = 5;

    int rand_x, rand_y;

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(1800,2200);
    while (cycle < cycles) {
        // random sampling
        if(rand()%100 >= 5){
            rand_x = distribution(generator);
            rand_y = distribution(generator);
        }else{
            rand_x = end_x;
            rand_y = end_y;
        }

        // Find the nearest node
        int nearestNode = findNearestNode(rand_x, rand_y);

        // expand the tree
        float theta = atan2(rand_y - NodeList[nearestNode].y, rand_x - NodeList[nearestNode].x)*180/3.14;

        int newNode_x = NodeList[nearestNode].x + stepsize*cos(theta);
        int newNode_y = NodeList[nearestNode].y + stepsize*sin(theta);
        float newNode_cost = NodeList[nearestNode].cost + stepsize;
        int newNode_parent = nearestNode;

        // outside the map
        if (newNode_x < 0 || newNode_y < 0 || newNode_x >= xs_ || newNode_y >= ys_)
            continue;
        // in the closed list
        if (potential[getIndex(newNode_x, newNode_y)] < POT_HIGH)
            continue;
        // obstacles
        if (costs[getIndex(newNode_x, newNode_y)] > 0)
            continue;
        if(costs[getIndex(newNode_x, newNode_y)]>=lethal_cost_)
            continue;

        // find near nodes
        int nearinds[200];
        float dlist[200];
        int nnearinds = 0;
        float r = 200.0*sqrt(log(NodeList.size())/NodeList.size());

        for(int i=0; i<NodeList.size();i++){
            if(pow(NodeList[i].x-newNode_x,2)+pow(NodeList[i].y-newNode_y,2)<pow(r,2)){
                nearinds[nnearinds] = i;
                nnearinds++;
            }
        }

        // choose best parent
        if(nnearinds!=0){
            for (int i = 0; i < nnearinds; ++i)
            {
                int dx = newNode_x-NodeList[nearinds[i]].x;
                int dy = newNode_y-NodeList[nearinds[i]].y;
                int d = sqrt(pow(dx,2)+pow(dy,2));
                float theta = atan2(dy,dx);
                // check collision
                dlist[i] = NodeList[nearinds[i]].cost+d;
                int tempNode_x = NodeList[nearinds[i]].x;
                int tempNode_y = NodeList[nearinds[i]].y;
                for (int j = 0; j < int(d/stepsize); ++j)
                {
                    tempNode_x += stepsize*cos(theta);
                    tempNode_y += stepsize*sin(theta);
                    if (costs[getIndex(tempNode_x, tempNode_y)])
                    {
                        dlist[i]=1e4;
                        break;
                    }
                }
            }
            // find the min cost
            float mincost = 1e4;
            int mindex = -1;
            for (int i = 0; i < nnearinds; ++i)
            {
                if (dlist[i]<mincost)
                {
                    mindex = i;
                    mincost = dlist[i];
                }
            }
            if (mincost < 1e4){
                newNode_cost = mincost;
                newNode_parent = nearinds[mindex];
            }
        }

//        potential[getIndex(newNode_x, newNode_y)] = potential[getIndex(NodeList[nearestNode].x, NodeList[nearestNode].y)] + 1;
        NodeList.push_back(Node(newNode_x, newNode_y, newNode_parent));
        NodeList.back().cost = newNode_cost;
        // rewire
        for (int i = 0; i < nnearinds; ++i)
        {
            int dx = newNode_x - NodeList[nearinds[i]].x;
            int dy = newNode_y - NodeList[nearinds[i]].y;
            int d = sqrt(pow(dx,2)+pow(dy,2));
            int scost = newNode_cost+d;

            if (NodeList[nearinds[i]].cost>scost){
                int theta = atan2(dy,dx);
                // check collision
                int tempNode_x = NodeList[nearinds[i]].x;
                int tempNode_y = NodeList[nearinds[i]].y;
                int flag = 1;
                for (int j = 0; j < int(d/stepsize); ++j)
                {
                    tempNode_x+=stepsize*cos(theta);
                    tempNode_y+=stepsize*sin(theta);
                    if (costs[getIndex(tempNode_x, tempNode_y)])
                    {
                        flag = 0;
                        break;
                    }
                }
                if(flag){
                    NodeList[nearinds[i]].parent = int(NodeList.size())-1;
                    NodeList[nearinds[i]].cost = scost;
                }
            }
        }

        if (sqrt(pow(newNode_x-end_x,2)+pow(newNode_y-end_y,2)) < stepsize){
            // generate the final path
            std::pair<float, float> current;
            current.first = end_x;
            current.second = end_y;
            path.push_back(current);
            int currentNode = NodeList.size()-1;
            while (currentNode>=0) {
                // ROS_INFO("%d %lf",currentNode,NodeList[currentNode].cost);
                current.first = NodeList[currentNode].x;
                current.second = NodeList[currentNode].y;
                path.push_back(current);
                currentNode = NodeList[currentNode].parent;
            }
        }
        cycle++;
    }
}

int RRTStar::findNearestNode(int x, int y)
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

void RRTStar::pathSmooth()
{

}
