#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/PathRequest.h>

int main(int argc, char* argv[]) {
    // check args first
    if (argc < 4) {
        std::cout << "Missing arguments" << std::endl;  // TODO: provide some info and instructions
        return 0;
    }

    // ROS init
    ros::init(argc, argv, "pathrequest_node");
    ros::NodeHandle nh;

    // Advertise for /pathreq topic
    ros::Publisher pathreq_pub = nh.advertise<nav_msgs::PathRequest>("pathreq", 1);

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

    // Parse grid size argument
    ss << argv[1];
    unsigned int gs;  // grid size
    ss >> gs;

    // Print out
    std::cout << "Start: (" << pathReq.start_x << ", " << pathReq.start_y << ")" << std::endl;
    std::cout << "Goal: (" << pathReq.goal_x << ", " << pathReq.goal_y << ")" << std::endl;

    // Publish to topic
    pathreq_pub.publish(pathReq);
}
