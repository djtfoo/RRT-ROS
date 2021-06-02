#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/PathRequest.h>

int main(int argc, char* argv[]) {
    // check args first
    if (argc <= 5) {
        std::cout << "Missing arguments; try running:" << std::endl;
        // provide info and instructions
        std::cout << "rosrun rrt_planner pathrequest_node <start_x> <start_y> <goal_x> <goal_y> <rrt_ver>" << std::endl;
        std::cout << " - [start/goal]_[xy]: pixel coordinates" << std::endl;
        std::cout << " - rrt_ver: 0 for basic RRT, 1 for RRT*" << std::endl;
        return 0;
    }

    // ROS init
    ros::init(argc, argv, "pathrequest_node");
    ros::NodeHandle nh;

    // Advertise for /pathreq topic
    ros::Publisher pathreq_pub = nh.advertise<nav_msgs::PathRequest>("pathreq", 1);

    ROS_INFO("Path Request node started");

    // Create message
    nav_msgs::PathRequest pathReq;

    // Parse arguments
    std::stringstream ss;
    ss << argv[1];
    ss >> pathReq.start_x;

    ss.clear();
    ss.str("");
    ss << argv[2];
    ss >> pathReq.start_y;

    ss.clear();
    ss.str("");
    ss << argv[3];
    ss >> pathReq.goal_x;

    ss.clear();
    ss.str("");
    ss << argv[4];
    ss >> pathReq.goal_y;

    ss.clear();
    ss.str("");
    ss << argv[5];
    ss >> pathReq.rrt_ver;

    // Publish to topic
    ros::Duration(1.0).sleep();
    pathreq_pub.publish(pathReq);

    ROS_INFO("Published to /pathreq");

    // Print out
    std::cout << "Start: (" << pathReq.start_x << ", " << pathReq.start_y << ")" << std::endl;
    std::cout << "Goal: (" << pathReq.goal_x << ", " << pathReq.goal_y << ")" << std::endl;
}
