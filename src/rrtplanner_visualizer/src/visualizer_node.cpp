#include <ros/ros.h>
#include <thread>

#include <nav_msgs/OccupancyGrid.h>
#include <rrt_planner/RrtNode.h>
#include <nav_msgs/Path.h>

#include "visualizer_window.h"
#include "msg_visualizer.h"

class VisualizerInterface {
public:
    VisualizerInterface(ros::NodeHandle& nh)
    {
        map_sub_ = nh.subscribe("map", 1, mapCallback);
        rrt_sub_ = nh.subscribe("rrtnode", 1, rrtnodeCallback);
        path_sub_ = nh.subscribe("path", 1, pathCallback);
    }
    ~VisualizerInterface() {
        if (window_ != nullptr)
           delete window_;
    }

    static void refreshWindow() {
        while (true) {
            if (window_ != nullptr)
                window_->displayWindow();
            ros::Duration(0.1).sleep();  // wait 100ms
        }
    }

private:
    // window
    static VisualizerWindow* window_;  // only 1 window

    // Subscriber
    ros::Subscriber map_sub_;  // subscribed to /map topic
    ros::Subscriber rrt_sub_;  // subscribed to /rrtnode topic
    ros::Subscriber path_sub_;  // subscribed to /path topic

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        // have a "Map Parser" to create VisualizerWindow and draw out the map
        MsgVisualizer::parseMap(&window_, map);
    }
    static void rrtnodeCallback(const rrt_planner::RrtNode::ConstPtr& rrtNode) {
        // have a "RrtNode Parser" to draw RRT nodes and edges on VisualizerWindow
        MsgVisualizer::parseRrtNode(&window_, rrtNode);
    }
    static void pathCallback(const nav_msgs::Path::ConstPtr& path) {
        // have a "Path Parser" to draw Path on VisualizerWindow
        MsgVisualizer::parsePath(&window_, path);
    }
};

VisualizerWindow* VisualizerInterface::window_ = nullptr;

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "visualizer_node");
    ros::NodeHandle nh;

    // Create Visualizer node's ROS interfaces
    VisualizerInterface visualizerInterface(nh);

    ROS_INFO("Visualizer node started");

    // Create visualizer thread
    std::thread th1(visualizerInterface.refreshWindow);

    // Don't exit the program
    ros::spin();
}
