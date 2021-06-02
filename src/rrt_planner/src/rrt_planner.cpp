#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>

#include <iostream> // testing only

#include "rrt_planner.h"
#include "rrt/rrt.h"

#define STATUS_REACHED 1
#define STATUS_ADVANCED 2
#define STATUS_TRAPPED 0

// Constructor
RrtPlanner::RrtPlanner(ros::NodeHandle& nh) {
    // advertise ROS topics
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
    rrt_pub_ = nh.advertise<nav_msgs::RrtNode>("rrtnode", 1);

    // init RNG with random seed
    srand(time(NULL));

    // wait some time after publishers get created
    ros::Duration(1.0).sleep();
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

// Path planner algorithm (RRT)
void RrtPlanner::planPath(nav_msgs::OccupancyGrid::ConstPtr map, const Coord& start, const Coord& goal) {

    ROS_INFO("Init Node.");

    // store reference to map (for convenience)
    map_ = map;

    // initialise RRT at start position
    Rrt node(start, nullptr);
    publishRrtNode(&node);

    // init unpopulated regions list
    unpopulatedRegions.clear();
    for (int i = 0; i < numRegionsX*numRegionsY; ++i) {
        unpopulatedRegions.push_back(i);
    }
    _incrementalStep = map_->gridsize - 1;

    // sleep for a short time
    ros::Duration(0.25).sleep();

    ROS_INFO("Build RRT.");

    // compute RRT
    Rrt* goalNode = buildRrt(&node, 10000, goal);
    if (goalNode != nullptr) {  // managed to reach the goal
        ROS_INFO("A path has been found.");
        // publish the path
        publishPath(goalNode);
    }
    else {
        ROS_INFO("No path from start to goal position found.");
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
    int i = 1;
    while (i <= iters) {
        std::cout << "Building RRT Node: " << i << std::endl;

        // sample random state
        Coord xRand;
        randomState(&xRand, currState->getCoord(), goal, 20);

        // extend RRT to xRand
        Rrt* xNew = nullptr;
        char status = extend(rrt, xRand, &xNew, goal);
        std::cout << "extend status: " << (int)status << std::endl;
        if (status != STATUS_TRAPPED) {  // node was created successfully
            xNew->setId(i);  // set node id
            // publish new node to ROS topic (for visualization)
            publishRrtNode(xNew);

            // update RRT state for sampling
            std::cout << "Update RRT state for randomState sampling" << std::endl;
            currState = xNew;
            Coord* newCoord = xNew->getCoord();
            // random sampling idea 3: large regions populations
            int xRegion = newCoord->_x / regionSizeX;
            int yRegion = newCoord->_y / regionSizeY;
            // remove region from unpopulated regions list
            std::vector<int>::iterator position = std::find(unpopulatedRegions.begin(), unpopulatedRegions.end(), xRegion*numRegionsX + yRegion);
            if (position != unpopulatedRegions.end()) { // == element was found
                unpopulatedRegions.erase(position);
                std::cout << "Removed region " << xRegion << "," << yRegion << " from list" << std::endl;
            }
            else
                std::cout << "Region was already removed" << std::endl;

            // check if goal is reached
            if (xNew->equalsState(goal)) {
               std::cout << "REACHED GOAL!!" << std::endl;
               return xNew;
            }


            // sleep for a short time
            ros::Duration(0.002).sleep();

            // no. nodes in RRT increased
            i += 1;
        }
        //if (status == STATUS_REACHED) // don't check this; this is extend's Reached state!
        //    return xNew;  // return RRT node at the goal
    }

    std::cout << iters << " iterations reached. Did not reach goal." << std::endl;

    return nullptr;  // did not manage to reach goal
}

void RrtPlanner::randomState(Coord* state, Coord* currState, const Coord& goal, int radius) {
    std::cout << "randomState: ";
    // biased coin: 10% probability of sampling goal
    int coin = rand() % 100;
    if (coin < 5) {
        state->_x = goal._x;
        state->_y = goal._y;
    }
    else {
        // have a high chance of sampling outside occupied regions (split map into multiple small regions)
        bool loop = true;
        while (loop) {
            if (coin < 80 && unpopulatedRegions.size() > 0) {
                // randomly pick a region to sample from
                int region = unpopulatedRegions[rand() % unpopulatedRegions.size()];
                int xRegion = region / numRegionsX;
                int yRegion = region % numRegionsX;

                // sample point within region
                Coord min(xRegion * regionSizeX, yRegion * regionSizeY);
                Coord max(min._x + regionSizeX, min._y + regionSizeY);
                state->_x = rand() % regionSizeX + min._x;
                state->_y = rand() % regionSizeY + min._y;
            }
            else {
                // idea 0: sample anywhere
                state->_x = rand() % map_->width;
                state->_y = rand() % map_->height;
            }
            loop = isObstacle(*state);  // is an obstacle: invalid state; continue loop
        }
    }
    std::cout << state->_x << ", " << state->_y << std::endl;
}

int RrtPlanner::extend(Rrt* rrt, const Coord& state, Rrt** xNew, const Coord& goal) {
    std::cout << "extend: " << std::endl;

    // find node in RRT that is nearest to the state
    Rrt* xNear = nearestNeighbour(rrt, state);
    std::cout << xNear->getCoord()->_x << ", " << xNear->getCoord()->_y << std::endl;

    std::cout << "xNear is node " << xNear->getId() << std::endl;

    // check if new state is valid
    Coord nState;
    if (newState(state, xNear, _incrementalStep, &nState, goal)) {
        std::cout << "newState is valid" << std::endl;
        std::cout << "newState:" << nState._x << ", " << nState._y << std::endl;
        // create new RRT node
        std::cout << "Create new node" << std::endl;
        *xNew = new Rrt(nState, xNear);
        // add new node to RRT
        xNear->addChild(*xNew);
        std::cout << "Added new node to RRT" << std::endl;
        if ((*xNew)->equalsState(state))
            return STATUS_REACHED;
        return STATUS_ADVANCED;
    }

    return STATUS_TRAPPED;
}

Rrt* RrtPlanner::nearestNeighbour(Rrt* rrt, const Coord& state) {
    std::cout << "nearestNeighbour: ";

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
    // TODO: should a state be on any pixel, or a continuous range?
    float vecX = state._x - xNear->getCoord()->_x;
    float vecY = state._y - xNear->getCoord()->_y;

    // change magnitude of vec to input
    double magnitude = sqrt(vecX*vecX + vecY*vecY);
    if (magnitude == 0.f)  // sampled pretty much the same point as xNear
        return false;

    // unit vector
    float unitVecX = vecX / magnitude;
    float unitVecY = vecY / magnitude;

    // if distance to xNear is too far, only move towards it by input
    if (magnitude >= input) {
        vecX = unitVecX * input;
        vecY = unitVecY * input;
    }

    // instead of clamping points within map boundary, do a collision check
    nState->_x = xNear->getCoord()->_x + vecX;
    nState->_y = xNear->getCoord()->_y + vecY;

    // also check more points along line, e.g. midpoint
    Coord midpoint(xNear->getCoord()->_x + unitVecX * 0.5*input,
        xNear->getCoord()->_y + unitVecY * 0.5*input);

    // check if new state is valid, i.e. input to new state does not intersect with obstacle
    return noCollision(*(xNear->getCoord()), *nState) && noCollision(*(xNear->getCoord()), midpoint);
}

bool RrtPlanner::isObstacle(const Coord& coord) {
    int gridX = coord._x / map_->gridsize;
    int gridY = coord._y / map_->gridsize;
    int gridWidth = map_->width / map_->gridsize;
    return map_->occupancy[gridY*gridWidth + gridX];
}

bool RrtPlanner::isObstacle(int gridX, int gridY) {
    int gridWidth = map_->width / map_->gridsize;
    return map_->occupancy[gridY*gridWidth + gridX];
}

bool RrtPlanner::noCollision(const Coord& startState, const Coord& newState) {

    // check if newState is out of bounds
    if (newState._x < 0 || newState._x >= map_->width ||
        newState._y < 0 || newState._y >= map_->height)
        return false;

    // check if the newState is an obstacle grid
    if (isObstacle(newState))
        return false;

    // Check that Manhattan distance from startState to newState exists
    // (special case: definitely only 2 grids wide to check, so just check the alternate corners)
    int start_gridX = startState._x / map_->gridsize;
    int start_gridY = startState._y / map_->gridsize;
    int new_gridX = newState._x / map_->gridsize;
    int new_gridY = newState._y / map_->gridsize;
    if (start_gridX != new_gridX && start_gridY != new_gridY) {
        if (isObstacle(start_gridX, newState._y) || isObstacle(new_gridX, start_gridY))
            return false;
    }

    // else, no collision
    return true;
}
