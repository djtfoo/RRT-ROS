#include <ros/ros.h>
#include "rrt_planner.h"

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    // Create Planner class for node's ROS interfaces and path planning
    RrtPlanner rrtPlanner(nh);

    ROS_INFO("Planner node started");

    // TODO: use argv to enable/disable visualization

    // Get start and end positions
    //Coord start(0, 0);
    //Coord end(450, 450);

    // Plan path
    //rrtPlanner.planPath(start, end);  // TODO: planPath should be a subscriber callback instead

    // Don't exit the program
    ros::spin();
}
