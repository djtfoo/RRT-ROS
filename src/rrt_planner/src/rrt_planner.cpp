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
ros::Publisher RrtPlanner::path_pub_;
ros::Publisher RrtPlanner::rrt_pub_;

// Constructor
RrtPlanner::RrtPlanner(ros::NodeHandle& nh) {
    // subscribe to ROS topics
    map_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("map", 1, mapCallback);
    // TODO: subscribe to start/goal position topic

    // advertise ROS topics
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
    rrt_pub_ = nh.advertise<rrt_planner::RrtNode>("rrtnode", 1);

    // init RNG with random seed
    srand(time(NULL));
}

// Publish path to ROS topic
void RrtPlanner::publishPath(const Rrt& goalNode) {

    nav_msgs::Path pathMsg;

    // TODO: populate pathMsg

    // publish pathMsg to topic
    path_pub_.publish(pathMsg);
}

// Subscriber callback
void RrtPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
    ROS_INFO("Map callback");
    map_ = map;

    // Set start and end positions
    Coord start(52.5, 52.5);
    Coord end(450, 450);

    // Plan path
    planPath(start, end);  // TODO: planPath should be a subscriber callback instead
}

// Path planner algorithm (RRT)
void RrtPlanner::planPath(const Coord& start, const Coord& goal) {

    ROS_INFO("Init Node.");

    // initialise RRT at start position
    Rrt node(start, nullptr);
    publishRrtNode(&node);

    // TODO: sleep only if visualization is true
    ros::Duration(0.25).sleep();

    ROS_INFO("Build RRT.");

    // compute RRT
    Rrt* goalNode = buildRrt(&node, 50);
    if (goalNode != nullptr) {  // managed to reach the goal
        // publish the path
        publishPath(*goalNode);
    }
}

void RrtPlanner::publishRrtNode(Rrt* node) {

    ROS_INFO("Publishing RRT Node.");
    std::cout << node->getId() << " | " << node->getCoord()->_x << "," << node->getCoord()->_y << std::endl;

    // Create RRT node message
    rrt_planner::RrtNode rrtnode;

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
Rrt* RrtPlanner::buildRrt(Rrt* rrt, int iters) {
    for (int i = 1; i <= iters; ++i) {
        // sample random state
        Coord xRand;
        randomState(&xRand);

        // extend RRT to xRand
        Rrt* xNew = nullptr;
        char status = extend(rrt, xRand, &xNew);
        if (status != STATUS_TRAPPED) {  // node was created successfully
            xNew->setId(i);  // set node id
            // TODO: publish only if visualization is true
            // publish new node to ROS topic
            publishRrtNode(xNew);
        }
        if (status == STATUS_REACHED) // success
            return xNew;  // return RRT node at the goal

        // TODO: sleep only if visualization is true
        ros::Duration(0.25).sleep();
    }

    return nullptr;  // did not manage to reach goal
}

void RrtPlanner::randomState(Coord* state) {
    // TODO: improve sampling method

    // naive: pick a random coordinate and return the middle coordinate of the grid

    // naive: pick a random pixel coordinate
    state->_x = rand() % (map_->width);
    state->_y = rand() % (map_->height);

    // TODO: check that it is not obstacle

}

int RrtPlanner::extend(Rrt* rrt, const Coord& state, Rrt** xNew) {
    // find node in RRT that is nearest to the state
    Rrt* xNear = nearestNeighbour(rrt, state);

    std::cout << "xNear is node " << xNear->getId() << std::endl;

    // check if new state is valid
    Coord nState;
    if (newState(state, xNear, _incrementalStep, &nState)) {
        // create new RRT node
        *xNew = new Rrt(nState, xNear);
        // add new node to RRT
        xNear->addChild(*xNew);
        if ((*xNew)->equalsState(state))
            return STATUS_REACHED;
        return STATUS_ADVANCED;
    }

    return STATUS_TRAPPED;

    /* TODO: modification of EXTEND: change to CONNECT */
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

bool RrtPlanner::newState(const Coord& state, Rrt* xNear, float input, Coord* nState) {

// TODO: repeatedly try possible states

    // TODO: should a state be on any pixel, or only a grid?
    float vecX = state._x - xNear->getCoord()->_x;
    float vecY = state._y - xNear->getCoord()->_y;

    // change magnitude of vec to input
    double magnitude = sqrt(vecX*vecX + vecY*vecY);
    vecX = (vecX / magnitude) * input;
    vecY = (vecY / magnitude) * input;

    nState->_x = xNear->getCoord()->_x + vecX;
    nState->_y = xNear->getCoord()->_y + vecY;

    // check if new state is valid, i.e. no collision detected
    return noCollision(state, *nState);
}

bool RrtPlanner::noCollision(const Coord& state, const Coord& newState) { 

    // check if the straight line from state to newState intersects with an obstacle grid
    int gridX = newState._x / map_->gridsize;
    int gridY = newState._y / map_->gridsize;
    int gridWidth = map_->width / map_->gridsize;
    if (map_->occupancy[gridY*gridWidth + gridX])
        return false;

    return true;
}
