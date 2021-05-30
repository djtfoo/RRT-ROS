#include <ros/ros.h>
#include <thread>
#include <iostream> // TESTING ONLY

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/RrtNode.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/PathRequest.h>

#include "visualizer_window.h"
#include "msg_visualizer.h"
#include "pathrequest/pathrequest_handler.h"

class PathRequestInterface {
public:
    PathRequestInterface(ros::NodeHandle& nh)
    {
        map_sub_ = nh.subscribe("map", 1, mapCallback);
        pathreq_pub_ = nh.advertise<nav_msgs::PathRequest>("pathreq", 1);
    }
    ~PathRequestInterface() {
        if (window_ != nullptr)
           delete window_;
    }

    static void refreshWindow() {
        while (true) {
            if (window_ != nullptr)
                window_->displayWindow();
            ros::Duration(0.01).sleep();  // wait 10ms
        }
    }

    static void processMouseEvent(int event, int mouseX, int mouseY, int flags, void* param) {

        // Set start grid
        if (event == EVENT_LBUTTONDOWN) {  // left mouse click
            // get grid coordinate
            int grid_x = mouseX / map_->gridsize;
            int grid_y = mouseY / map_->gridsize;

            // check if grid is valid
            if (!PathRequestHandler::isObstacle(map_, grid_x, grid_y) &&  // selected grid is not an obstacle
                (grid_x != goalX_ && grid_y != goalY_))  // does not overlap with goal grid
            {
                // clear fill of previous grid
                if (startX_ != -1 && startY_ != -1)
                    PathRequestHandler::clearGrid(window_, startX_, startY_, map_->gridsize);

                // save start grid coordinates
                startX_ = grid_x;
                startY_ = grid_y;

                // draw fill for selected grid
                PathRequestHandler::fillStartGrid(window_, grid_x, grid_y, map_->gridsize);
            }
        }
        // Set goal grid
        else if (event == EVENT_RBUTTONDOWN) {  // right mouse click
            // get grid coordinate
            int grid_x = mouseX / map_->gridsize;
            int grid_y = mouseY / map_->gridsize;

            // check if grid is valid
            if (!PathRequestHandler::isObstacle(map_, grid_x, grid_y) &&  // selected grid is not an obstacle
                (grid_x != startX_ && grid_y != startY_))  // does not overlap with start grid
            {
                // clear fill of previous grid
                if (goalX_ != -1 && goalY_ != -1)
                    PathRequestHandler::clearGrid(window_, goalX_, goalY_, map_->gridsize);

                // save start grid coordinates
                goalX_ = grid_x;
                goalY_ = grid_y;

                // draw fill for selected grid
                PathRequestHandler::fillGoalGrid(window_, grid_x, grid_y, map_->gridsize);
            }
        }
    }

private:
    // window
    static VisualizerWindow* window_;  // only 1 window

    // Subscriber
    ros::Subscriber map_sub_;  // subscribed to /map topic
    // Subscriber messages
    static nav_msgs::OccupancyGrid::ConstPtr map_;
    // Publisher
    ros::Publisher pathreq_pub_;  // publisher for /pathreq topic
    // Message data
    static int startX_;
    static int startY_;
    static int goalX_;
    static int goalY_;

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        // store reference to map data
        map_ = map;
        // have a "Map Parser" to create VisualizerWindow and draw out the map
        MsgVisualizer::parseMap(&window_, map, false);
        // set mouse click event callback
        window_->setMouseCallbackFunc(processMouseEvent);
    }
};

VisualizerWindow* PathRequestInterface::window_ = nullptr;
nav_msgs::OccupancyGrid::ConstPtr PathRequestInterface::map_ = nullptr;
int PathRequestInterface::startX_ = -1;
int PathRequestInterface::startY_ = -1;
int PathRequestInterface::goalX_ = -1;
int PathRequestInterface::goalY_ = -1;

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "pathrequest_node");
    ros::NodeHandle nh;

    // Create this node's ROS interfaces
    PathRequestInterface pathRequestInterface(nh);

    ROS_INFO("Path Request node started");

    // Create visualizer thread
    std::thread th1(pathRequestInterface.refreshWindow);

    // Don't exit the program
    ros::spin();
}
