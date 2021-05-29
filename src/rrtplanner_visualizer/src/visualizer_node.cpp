#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <rrt_planner/RrtNode.h>

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

    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {

    }
    static void rrtnodeCallback(const rrt_planner::RrtNode::ConstPtr& rrtNode) {

    }
};

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "visualizer_node");
    ros::NodeHandle nh;

    // Create Visualizer node's ROS interfaces
    RrtPlanner rrtPlanner(nh);

    ROS_INFO("Visualizer node started");

    // Don't exit the program
    ros::spin();
}
