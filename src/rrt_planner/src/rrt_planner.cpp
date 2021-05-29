#include <ros/ros.h>
#include <stdlib.h>

#include "rrt_planner.h"
#include "rrt/rrt.h"

#define STATUS_REACHED 1
#define STATUS_ADVANCED 2
#define STATUS_TRAPPED 0

nav_msgs::OccupancyGrid::ConstPtr RrtPlanner::map_ = nullptr;

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
}

// Path planner algorithm (RRT)
void RrtPlanner::planPath(const Coord& start, const Coord& goal) {

    // initialise RRT at start position
    Rrt node(start, nullptr);

    // compute RRT
    Rrt* goalNode = buildRrt(&node, 200);
    if (goalNode != nullptr) {  // managed to reach the goal
        // publish the path
        publishPath(*goalNode);
    }
}

void RrtPlanner::publishRrtNode(Rrt* node) {

    // Create RRT node message
    rrt_planner::RrtNode rrtnode;
    rrtnode.id = node->getId();
    rrtnode.x = node->getCoord()->_x;
    rrtnode.y = node->getCoord()->_y;
    rrtnode.parent = node->getParent()->getId();

    // publish RRT node message
    rrt_pub_.publish(rrtnode);
}

// RRT helper functions
Rrt* RrtPlanner::buildRrt(Rrt* rrt, int iters) {
    for (int i = 0; i < iters; ++i) {
        // sample random state
        Coord xRand;
        randomState(&xRand);

        // extend RRT to xRand
        Rrt* xNew = nullptr;
        char status = extend(rrt, xRand, &xNew);
        if (status != STATUS_TRAPPED)  // node was created successfully
            xNew->setId(i);  // set node id
        if (status == STATUS_REACHED) // success
            return xNew;  // return RRT node at the goal

        // TODO: publish only if visualization is true
        // publish new node to ROS topic
        publishRrtNode(xNew);

        // TODO: sleep only if visualization is true
        ros::Duration(0.1).sleep();  // sleep for 0.1s
    }

    return nullptr;  // did not manage to reach goal
}

void RrtPlanner::randomState(Coord* state) {
    // TODO: improve sampling method

    // naive: pick a random coordinate and return the middle coordinate of the grid

    // naive: pick a random pixel coordinate
    state->_x = rand() % (map_->width * map_->gridsize);
    state->_y = rand() % (map_->height * map_->gridsize);

    // TODO: check that it is not obstacle

}

int RrtPlanner::extend(Rrt* rrt, const Coord& state, Rrt** xNew) {
    // find node in RRT that is nearest to the state
    Rrt* xNear = nearestNeighbour(rrt, state);

    // check if new state is valid
    Coord nState;
    if (newState(state, xNear, incrementalStep, &nState)) {
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

    Rrt* nearestNeighbour = rrt;

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
    for (int i = 0; i < rrt->getNumChildren(); ++i) {
        Rrt* child = rrt->getChild(i);
        float childDist;
        nearestNeighbourSearch(child, state, &childDist);
        if (childDist < *dist) {
            nearestNeighbour = child;
            *dist = childDist;
        }
    }

    return nearestNeighbour;
}

float RrtPlanner::neighbourDistanceMetric(Rrt* rrt, const Coord& state) {
    // distance metric is square of magnitude
    float xLen = rrt->getCoord()->_x - state._x;
    float yLen = rrt->getCoord()->_y - state._y;
    return (xLen*xLen + yLen*yLen);
}

bool RrtPlanner::newState(const Coord& state, Rrt* xNear, float input, Coord* nState) {

    // + incrementalStep
    // TODO: should a state be on any pixel, or only a grid?

    // check if new state is valid, i.e. no collision detected
    return hasCollision(state, *nState);
}

bool RrtPlanner::hasCollision(const Coord& state, const Coord& newState) { 

    // check if the straight line from state to newState intersects with an obstacle grid

    return false;
}
