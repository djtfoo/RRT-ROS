#ifndef PLANNER_H_
#define PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid>
#include <nav_msgs/Path>

#include <vector>

class Planner
{
public:
    // Constructor
    Planner(ros::NodeHandle& nh);

private:
    // Subscriber
    ros::Subscriber map_sub_;

    // Subscriber callback
    //callback func

    // Publisher
    ros::Publisher path_pub_;

protected:
    // plan path
    virtual void plan_path() = 0;

    // publish path
    void publish_path(std::vector<Coord2D>& path);
};

#endif	// PLANNER_H_
