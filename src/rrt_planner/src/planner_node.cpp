#include <ros/ros.h>
#include "planner.h"

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    // Create Planner class for node's ROS interfaces and path planning
    Planner planner(nh);

    ROS_INFO("Planner node started");

    // Don't exit the program
    ros::spin();
}
