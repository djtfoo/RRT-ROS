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

// for RRT state sampling
float RrtPlanner::minX = 0.f;  // inclusive
float RrtPlanner::minY = 0.f;  // inclusive
float RrtPlanner::maxX = 0.f;  // inclusive
float RrtPlanner::maxY = 0.f;  // inclusive
/*int RrtPlanner::populationCount[10][10] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};*/

std::vector<int> RrtPlanner::unpopulatedRegions;

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

    maxX = map->width - 1;   // inclusive
    maxY = map->height - 1;  // inclusive

    // Set start and end positions
    Coord start(50, 50);
    Coord end(50, 480);

    // Plan path
    planPath(start, end);  // TODO: planPath should be a subscriber callback instead
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

    // TODO: sleep only if visualization is true
    ros::Duration(0.25).sleep();

    ROS_INFO("Build RRT.");

    // compute RRT
    Rrt* goalNode = buildRrt(&node, 6000, goal);
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
            // TODO: publish only if visualization is true
            // publish new node to ROS topic
            publishRrtNode(xNew);

            // update RRT state for sampling
            currState = xNew;
            Coord* newCoord = xNew->getCoord();
            // idea 2: bounding box
            //minX = fmin(newCoord->_x, minX);
            //minY = fmin(newCoord->_y, minY);
            //maxX = fmax(newCoord->_x, maxX);
            //maxY = fmax(newCoord->_y, maxY);
            // idea 3: large regions populations
            int xRegion = newCoord->_x / regionSizeX;
            int yRegion = newCoord->_y / regionSizeY;
            //populationCount[xRegion][yRegion] += 1;
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
    // TODO: improve random sampling method - simple ways of approximating Voronoi region
    // idea 1: split map into large regions (e.g. 4 squares), higher probability bias towards sampling points within regions with fewer no. points (proportional probability)
    // idea 2: within curr state's radius, split into directional regions, same as idea 1 about higher probability bias (i.e. bias towards a certain direction)
    // idea 3: track bounding box of RRT, very strong bias towards sampling outside the bounding box (maybe just a region outside the bounding box?)
    // problem: these methods are not encompassing the fact that using Voronoi regions can sample the smaller areas in-between branches - but then again, I guess this is not a problem for pathfinding towards a goal

  // instead of clamping, how about perform collision check and reject the state config?

    // biased coin: 5% probability of sampling goal
    int coin = rand() % 100;
    if (coin < 10) {
        state->_x = goal._x;
        state->_y = goal._y;
    }
    else {
        // idea 4: based on idea 2 - have a high chance of sampling outside occupied regions (split map into multiple small regions)

        // TODO: Problem: inaccessible areas in map count towards unpopulated regions
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

        // idea 3: split map into large regions

        // idea 2: bias towards sampling outside RRT bounding box -- idea is decent, but it should have a higher fidelity than a box
        /*if (minX > 0 && maxX < map_->width - 1 &&
            minY > 0 && maxY < map_->height - 1 &&
            coin < 85)
        {
            int x = minX, y = minY;
            while (x >= minX && x <= maxX && y >= minY && y <= maxY) {
                // ver 1: sample within a region outside the bounding box
                //x = rand() % (int)(maxX - minX + radius) + minX - 0.5f*radius;
                //y = rand() % (int)(maxY - minY + radius) + minY - 0.5f*radius;
                // ver 2: sample anywhere outside the bounding box
                x = rand() % (map_->width);
                y = rand() % (map_->height);
            }
            state->_x = x;
            state->_y = y;
        }
        else {  // sample within RRT bounding box
            int x = rand() % (int)(maxX - minX) + minX;
            int y = rand() % (int)(maxY - minY) + minY;
            state->_x = x;
            state->_y = y;
        }*/

        // idea 1: Sample within a small space around current state -- gets stuck
/*label:
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
            goto label;*/

        // idea 0: naive random sampling, just pick a random pixel coordinate -- tree able to grow, but far from optimal, and is slow
        //state->_x = rand() % (map_->width);
        //state->_y = rand() % (map_->height);

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
    //}

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
