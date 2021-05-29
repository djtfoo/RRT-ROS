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
    void planPath(const Coord& start, const Coord& goal);

protected:
    // RRT stuff
    //probably helper funcs to do stuff like compute Voronoi region, get config, collision detection, etc
    Rrt* buildRrt(Rrt* rrt, int iters);
    void randomState(Coord* state);
    int extend(Rrt* rrt, const Coord& state, Rrt** xNew);

    Rrt* nearestNeighbour(Rrt* rrt, const Coord& state);
    Rrt* nearestNeighbourSearch(Rrt* rrt, const Coord& state, float* dist);
    float neighbourDistanceMetric(Rrt* rrt, const Coord& state);

    bool newState(const Coord& state, Rrt* xNear, float input, Coord* nState);
    bool hasCollision(const Coord& state, const Coord& newState);

    // publish path
    void publishPath(const Rrt& goalNode);

private:
    // Subscriber
    ros::Subscriber map_sub_;
    // TODO: ros::Subscriber for start/goal position topic

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
    static nav_msgs::OccupancyGrid::ConstPtr map_;

    // Publisher
    ros::Publisher path_pub_;
    ros::Publisher rrt_pub_;

    // Publisher function
    void publishRrtNode(Rrt* node);

    // RRT stuff
    float incrementalStep;  // represents input to next state (TBD: change to a class to accommodate nonholonomy)
};

#endif	// RRT_PLANNER_H_
