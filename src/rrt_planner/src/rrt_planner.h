#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/PathNode.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <vector>

#include "rrt/coord.h"

class RrtPlanner
{
public:
    // Constructor
    RrtPlanner(ros::NodeHandle& nh);

protected:
    // plan path
    void planPath(const Coord& start, const Coord& end);

    // RRT stuff
    //probably helper funcs to do stuff like compute Voronoi region, get config, collision detection, etc

    // publish path
    void publishPath(std::vector<nav_msgs::PathNode>& path);

private:
    // Subscriber
    ros::Subscriber map_sub_;

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);

    // Publisher
    ros::Publisher path_pub_;
};

#endif	// RRT_PLANNER_H_
