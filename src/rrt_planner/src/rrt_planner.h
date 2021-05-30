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
    static int extend(Rrt* rrt, const Coord& state, Rrt** xNew);

    static Rrt* nearestNeighbour(Rrt* rrt, const Coord& state);
    static Rrt* nearestNeighbourSearch(Rrt* rrt, const Coord& state, float* dist);
    static float neighbourDistanceMetric(Rrt* rrt, const Coord& state);

    static bool newState(const Coord& state, Rrt* xNear, float input, Coord* nState);
    static bool noCollision(const Coord& newState);

    // publish path
    static void publishPath(const Rrt& goalNode);

private:
    // Subscriber
    static ros::Subscriber map_sub_;
    // TODO: ros::Subscriber for start/goal position topic

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    static nav_msgs::OccupancyGrid::ConstPtr map_;

    // Publisher
    static ros::Publisher path_pub_;
    static ros::Publisher rrt_pub_;

    // Publisher function
    static void publishRrtNode(Rrt* node);

    // RRT stuff
    static float _incrementalStep;  // represents input to next state (TBD: change to a class to accommodate nonholonomy)
};

#endif	// RRT_PLANNER_H_
