#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <rrt_planner/RrtNode.h>

#include "visualizer_window.h"

using namespace cv;

class VisualizerInterface {
public:
    VisualizerInterface(ros::NodeHandle& nh) {
        map_sub_ = nh.subscribe("map", 1, mapCallback);
        rrt_sub_ = nh.subscribe("rrtnode", 1, rrtnodeCallback);
    }

private:
    // Subscriber
    ros::Subscriber map_sub_;  // subscribed to /map topic
    ros::Subscriber rrt_sub_;  // subscribed to /rrtnode topic

    // have a "MapParser" class to create VisualizerWindow and draw out the map

    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {

    }

    // have a "RrtNodeParser" class to draw RRT nodes and edges on VisualizerWindow

    static void rrtnodeCallback(const rrt_planner::RrtNode::ConstPtr& rrtNode) {

    }
};

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "visualizer_node");
    ros::NodeHandle nh;

    // Create Visualizer node's ROS interfaces
    VisualizerInterface visualizerInterface(nh);

    ROS_INFO("Visualizer node started");

    // Don't exit the program
    ros::spin();
}
