#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/PathNode.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <rrt_planner/RrtNode.h>

#include <vector>

#include "rrt/rrt.h"
#include "rrt/coord.h"

class RrtPlanner
{
public:
    // Constructor
    RrtPlanner(ros::NodeHandle& nh);

    // TODO: plan path could be protected/private as it should be from a subscriber callback
    static void planPath(const Coord& start, const Coord& goal);

protected:
    // RRT helper functions to do stuff like compute Voronoi region, get config, collision detection, etc
    static Rrt* buildRrt(Rrt* rrt, int iters, const Coord& goal);
    static void randomState(Coord* state, Coord* currState, const Coord& goal, int radius);
    static int extend(Rrt* rrt, const Coord& state, Rrt** xNew, const Coord& goal);

    static Rrt* nearestNeighbour(Rrt* rrt, const Coord& state);
    static Rrt* nearestNeighbourSearch(Rrt* rrt, const Coord& state, float* dist);
    static float neighbourDistanceMetric(Rrt* rrt, const Coord& state);

    static bool newState(const Coord& state, Rrt* xNear, float input, Coord* nState, const Coord& goal);
    static bool noCollision(const Coord& newState);

    // publish path
    static void publishPath(const Rrt& goalNode);

private:
    // Subscriber
    static ros::Subscriber map_sub_;
    // TODO: ros::Subscriber for start/goal position topic, and
    // make start and end static variables instead of constantly passing it as a variable

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    static nav_msgs::OccupancyGrid::ConstPtr map_;

    // Publisher
    static ros::Publisher path_pub_;
    static ros::Publisher rrt_pub_;

    // Publisher function
    static void publishRrtNode(Rrt* node);

    // RRT stuff
    //static Coord start;
    //static Coord end;
    static float _incrementalStep;  // represents input to next state (TBD: change to a class to accommodate nonholonomy/more complicated inputs)

    // random sampling idea 2: bounding box
    static float minX;
    static float minY;
    static float maxX;
    static float maxY;
    // random sampling idea 3: large region population
    const static int numRegionsX = 10;
    const static int numRegionsY = 10;
    const static int regionSizeX = 50;
    const static int regionSizeY = 50;
    //static int populationCount[10][10];
    static std::vector<int> unpopulatedRegions;
};

#endif	// RRT_PLANNER_H_
