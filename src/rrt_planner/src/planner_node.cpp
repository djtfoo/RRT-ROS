#include <ros/ros.h>
#include "rrt_planner.h"

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    // Create Planner class for node's ROS interfaces and path planning
    RrtPlanner rrtPlanner(nh);

    ROS_INFO("Planner node started");

    // Don't exit the program
    ros::spin();
}
