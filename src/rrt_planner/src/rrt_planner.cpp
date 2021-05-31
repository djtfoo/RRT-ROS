#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>

#include <iostream> // testing only

#include "rrt_planner.h"
#include "rrt/rrt.h"

#define STATUS_REACHED 1
#define STATUS_ADVANCED 2
#define STATUS_TRAPPED 0

nav_msgs::OccupancyGrid::ConstPtr RrtPlanner::map_ = nullptr;
float RrtPlanner::_incrementalStep = 4.f;

ros::Subscriber RrtPlanner::map_sub_;
ros::Subscriber RrtPlanner::pathreq_sub_;
ros::Publisher RrtPlanner::path_pub_;
ros::Publisher RrtPlanner::rrt_pub_;

// for RRT state sampling
std::vector<int> RrtPlanner::unpopulatedRegions;

// Constructor
RrtPlanner::RrtPlanner(ros::NodeHandle& nh) {
    // subscribe to ROS topics
    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, mapCallback);
    pathreq_sub_ = nh.subscribe<nav_msgs::PathRequest>("pathreq", 1, pathreqCallback);

    // advertise ROS topics
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
    rrt_pub_ = nh.advertise<nav_msgs::RrtNode>("rrtnode", 1);

    // init RNG with random seed
    srand(time(NULL));
}

// Publish path to ROS topic
void RrtPlanner::publishPath(Rrt* goalNode) {

    ROS_INFO("Publishing Path ...");

    // populate path in pathMsg
    std::cout << "Path from goal: ";
    std::vector<nav_msgs::RrtNode> path;
    Rrt* trace = goalNode;
    while (trace->getParent() != nullptr) {
        // create node
        nav_msgs::RrtNode node;
        node.id = trace->getId();
        node.x = trace->getCoord()->_x;
        node.y = trace->getCoord()->_y;
        node.parent = trace->getParent()->getId();
        // insert node to path
        path.push_back(node);
        std::cout << trace->getId() << " ";
        // go to next node
        trace = trace->getParent();
    }
    // create root node
    nav_msgs::RrtNode node;
    node.id = trace->getId();
    node.x = trace->getCoord()->_x;
    node.y = trace->getCoord()->_y;
    node.parent = -1;
    // insert root node to path
    path.push_back(node);
    std::cout << trace->getId() << std::endl;

    // assign created path
    nav_msgs::Path pathMsg;
    pathMsg.path = path;

    // publish pathMsg to topic
    path_pub_.publish(pathMsg);

    ROS_INFO("Published Path.");
}

// Subscriber callback
void RrtPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    ROS_INFO("Map message received");

    // Save map
    map_ = map;
}

void RrtPlanner::pathreqCallback(const nav_msgs::PathRequest::ConstPtr& pathreq) {
    ROS_INFO("Path Request message received");

    // Set start and end positions
    Coord start(pathreq->start_x, pathreq->start_y);
    Coord end(pathreq->goal_x, pathreq->goal_y);

    // TODO: Validate start and end coordinates first (command line publisher did not validate them)

    // Plan path
    planPath(start, end);
}

// Path planner algorithm (RRT)
void RrtPlanner::planPath(const Coord& start, const Coord& goal) {

    ROS_INFO("Init Node.");

    // initialise RRT at start position
    Rrt node(start, nullptr);
    publishRrtNode(&node);

    // init unpopulated regions list
    for (int i = 0; i < numRegionsX*numRegionsY; ++i) {
        unpopulatedRegions.push_back(i);
    }
    _incrementalStep = map_->gridsize - 1;

    // TODO: sleep only if visualization is true
    ros::Duration(0.25).sleep();

    ROS_INFO("Build RRT.");

    // compute RRT
    Rrt* goalNode = buildRrt(&node, 10000, goal);
    if (goalNode != nullptr) {  // managed to reach the goal
        // publish the path
        publishPath(goalNode);
    }
}

void RrtPlanner::publishRrtNode(Rrt* node) {

    ROS_INFO("Publishing RRT Node ...");
    std::cout << node->getId() << " | " << node->getCoord()->_x << "," << node->getCoord()->_y << std::endl;

    // Create RRT node message
    nav_msgs::RrtNode rrtnode;

    rrtnode.id = node->getId();
    rrtnode.x = node->getCoord()->_x;
    rrtnode.y = node->getCoord()->_y;
    if (node->getParent() == nullptr)
        rrtnode.parent = -1;
    else
        rrtnode.parent = node->getParent()->getId();

    // publish RRT node message
    rrt_pub_.publish(rrtnode);

    ROS_INFO("Published RRT Node.");
}

// RRT helper functions
Rrt* RrtPlanner::buildRrt(Rrt* rrt, int iters, const Coord& goal) {

    Rrt* currState = rrt;
    for (int i = 1; i <= iters; ++i) {
    if (currState == nullptr) {
        ROS_INFO("currState NULL");
    }
        // sample random state
        Coord xRand;
        randomState(&xRand, currState->getCoord(), goal, 20);

        // extend RRT to xRand
        Rrt* xNew = nullptr;
        char status = extend(rrt, xRand, &xNew, goal);
        if (status != STATUS_TRAPPED) {  // node was created successfully
            xNew->setId(i);  // set node id
            // publish new node to ROS topic (for visualization)
            publishRrtNode(xNew);

            // update RRT state for sampling
            currState = xNew;
            Coord* newCoord = xNew->getCoord();
            // random sampling idea 3: large regions populations
            int xRegion = newCoord->_x / regionSizeX;
            int yRegion = newCoord->_y / regionSizeY;
            // remove region from unpopulated regions list
            std::vector<int>::iterator position = std::find(unpopulatedRegions.begin(), unpopulatedRegions.end(), xRegion*numRegionsX + yRegion);
            if (position != unpopulatedRegions.end()) // == element was found
                unpopulatedRegions.erase(position);

            // check if goal is reached
            if (xNew->equalsState(goal))
               return xNew;

        }
        //if (status == STATUS_REACHED) // don't check this; this is extend's Reached state!
        //    return xNew;  // return RRT node at the goal

        // TODO: sleep only if visualization is true
        ros::Duration(0.005).sleep();
    }

    return nullptr;  // did not manage to reach goal
}

void RrtPlanner::randomState(Coord* state, Coord* currState, const Coord& goal, int radius) {
    // biased coin: 10% probability of sampling goal
    int coin = rand() % 100;
    if (coin < 10) {
        if (coin < 8) {  // sample goal - high chance
            state->_x = goal._x;
            state->_y = goal._y;
        }
        else {  // TODO: determine if necessary to sample around goal zone
            bool loop = true;
            while (loop) {
                int x = rand() % 5 - 3;
                int y = rand() % 5 - 3;
                state->_x = currState->_x + x;
                state->_y = currState->_y + y;
                loop = !noCollision(*state);
            }
        }
    }
    else {
        // have a high chance of sampling outside occupied regions (split map into multiple small regions)
        if (coin < 80)
        {
            // randomly pick a region to sample from
            int region = unpopulatedRegions[rand() % unpopulatedRegions.size()];
            int xRegion = region / numRegionsX;
            int yRegion = region % numRegionsX;

            // sample point within region
            Coord min(xRegion * regionSizeX, yRegion * regionSizeY);
            Coord max(min._x + regionSizeX, min._y + regionSizeY);
            int x = rand() % regionSizeX + min._x;
            int y = rand() % regionSizeY + min._y;

            state->_x = x;
            state->_y = y;
        }
        else {
// idea 1: Sample within a small space around current state
        if (unpopulatedRegions.size() < 20) { // most regions explored
            // idea 0: sample anywhere
            state->_x = rand() % map_->width;
            state->_y = rand() % map_->height;
        }
        else {
label:
            int x = 0, y = 0;
            while (x == 0 && y == 0) {
                x = rand() % radius - 0.5f*radius;
                y = rand() % radius - 0.5f*radius;
            }
            state->_x = currState->_x + x;
            state->_y = currState->_y + y;
            //state->_x = fmin(fmax(currState->_x + x, 0), map_->width - 1);
            //state->_y = fmin(fmax(currState->_y + y, 0), map_->height - 1);
            if (!noCollision(*state))
                goto label;
            }
        }

        // TODO: check that it is not obstacle?

    }

}

int RrtPlanner::extend(Rrt* rrt, const Coord& state, Rrt** xNew, const Coord& goal) {
    // find node in RRT that is nearest to the state
    Rrt* xNear = nearestNeighbour(rrt, state);

    std::cout << "xNear is node " << xNear->getId() << std::endl;

    // check if new state is valid
    Coord nState;
    if (newState(state, xNear, _incrementalStep, &nState, goal)) {
        // create new RRT node
        *xNew = new Rrt(nState, xNear);
        // add new node to RRT
        xNear->addChild(*xNew);
        if ((*xNew)->equalsState(state))
            return STATUS_REACHED;
        return STATUS_ADVANCED;
    }

    return STATUS_TRAPPED;
}

Rrt* RrtPlanner::nearestNeighbour(Rrt* rrt, const Coord& state) {

    // visit all nodes recursively to find nearest neighbour
    float dist;  // not used
    return nearestNeighbourSearch(rrt, state, &dist);
}

Rrt* RrtPlanner::nearestNeighbourSearch(Rrt* rrt, const Coord& state, float* dist) {

    *dist = neighbourDistanceMetric(rrt, state);

    // leaf node
    if (rrt->getNumChildren() == 0)
        return rrt;

    // find either (grand-)child or current node that is nearest to state
    Rrt* nearestNeighbour = rrt;
    float childDist;
    for (int i = 0; i < rrt->getNumChildren(); ++i) {
        Rrt* child = rrt->getChild(i);
        Rrt* childNearestNeighbour = nearestNeighbourSearch(child, state, &childDist);
        if (childDist < *dist) {
            nearestNeighbour = childNearestNeighbour;
            *dist = childDist;
        }
    }

    return nearestNeighbour;
}

float RrtPlanner::neighbourDistanceMetric(Rrt* rrt, const Coord& state) {
    // distance metric is square of magnitude
    float xLen = 0.01f * (rrt->getCoord()->_x - state._x);
    float yLen = 0.01f * (rrt->getCoord()->_y - state._y);
    return (xLen*xLen + yLen*yLen);
}

bool RrtPlanner::newState(const Coord& state, Rrt* xNear, float input, Coord* nState, const Coord& goal) {

// TODO: repeatedly try possible states

    // TODO: should a state be on any pixel, or only a grid?
    float vecX = state._x - xNear->getCoord()->_x;
    float vecY = state._y - xNear->getCoord()->_y;

    // change magnitude of vec to input
    double magnitude = sqrt(vecX*vecX + vecY*vecY);
    if (magnitude < 0.01f)  // sampled the same point as current state
        return false;
    else if (magnitude >= input) {
        vecX = (vecX / magnitude) * input;
        vecY = (vecY / magnitude) * input;
    }

    // instead of clamping points within map boundary, do a collision check
    nState->_x = xNear->getCoord()->_x + vecX;
    nState->_y = xNear->getCoord()->_y + vecY;

    // check if new state is valid, i.e. no collision detected
    return noCollision(*nState);
}

bool RrtPlanner::noCollision(const Coord& newState) {

    // check if newState is out of bounds
    if (newState._x < 0 || newState._x >= map_->width ||
        newState._y < 0 || newState._y >= map_->height)
        return false;

    // check if the straight line from state to newState intersects with an obstacle grid
    int gridX = newState._x / map_->gridsize;
    int gridY = newState._y / map_->gridsize;
    int gridWidth = map_->width / map_->gridsize;
    if (map_->occupancy[gridY*gridWidth + gridX])
        return false;

    // else, no collision
    return true;
}
