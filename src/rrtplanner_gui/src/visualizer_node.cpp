#include <ros/ros.h>
#include <thread>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/PathRequest.h>
#include <nav_msgs/RrtNode.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/PathRequest.h>

#include "visualizer_window.h"
#include "msg_handler.h"
#include "pathrequest/pathrequest_handler.h"

#include "gui/button.h"

class VisualizerInterface {
public:
    VisualizerInterface(ros::NodeHandle& nh)
    {
        // Subscribe to ROS topics
        map_sub_ = nh.subscribe("map", 1, mapCallback);
        pathreq_sub_ = nh.subscribe("pathreq", 1, pathreqCallback);
        rrt_sub_ = nh.subscribe("rrtnode", 1, rrtnodeCallback);
        path_sub_ = nh.subscribe("path", 1, pathCallback);

        // Advertise ROS topics
        pathreq_pub_ = nh.advertise<nav_msgs::PathRequest>("pathreq", 1);

        // add Button
        Button::createButton(Anchor(BottomLeft, 50, 5), 100, 30, 180, 180, 180, "Plan Path", (void*)publishPathRequest);
    }
    ~VisualizerInterface() {
        if (window_ != nullptr)
           delete window_;
    }

    static void refreshWindow() {
        while (true) {
            if (window_ != nullptr)
                window_->displayWindow();
            ros::Duration(0.002).sleep();  // wait 2ms
        }
    }

private:
    // window
    static VisualizerWindow* window_;  // only 1 window

    // Subscriber
    ros::Subscriber map_sub_;  // subscribed to /map topic
    ros::Subscriber pathreq_sub_;  // subscribed to /pathreq topic
    ros::Subscriber rrt_sub_;  // subscribed to /rrtnode topic
    ros::Subscriber path_sub_;  // subscribed to /path topic

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        // store reference to map data
        map_ = map;
        // have a "Map Parser" to create VisualizerWindow and draw out the map
        MsgHandler::parseMap(&window_, map, false);
        // set mouse click event callback
        window_->setMouseCallbackFunc(processMouseEvent);
    }
    static void pathreqCallback(const nav_msgs::PathRequest::ConstPtr& pathreq) {
        if (window_ != nullptr) {
            // clear start and goal nodes
            //cleanStartGoalPos();
            // clear everything by drawing out the map again from scratch
            MsgHandler::parseMap(&window_, map_, false);
            // set mouse click event callback
            window_->setMouseCallbackFunc(processMouseEvent);
            // have a "Path Request Parser" to draw start and goal nodes on VisualizerWindow
            MsgHandler::parsePathRequest(window_, pathreq);
        }
        else
            ROS_INFO("Received message from /pathreq, but no window is currently open.");
    }
    static void rrtnodeCallback(const nav_msgs::RrtNode::ConstPtr& rrtNode) {
        // have a "RrtNode Parser" to draw RRT nodes and edges on VisualizerWindow
        if (window_ != nullptr)
            MsgHandler::parseRrtNode(window_, rrtNode);
        else
            ROS_INFO("Received message from /rrtnode, but no window is currently open.");
    }
    static void pathCallback(const nav_msgs::Path::ConstPtr& path) {
        // have a "Path Parser" to draw Path on VisualizerWindow
        if (window_ != nullptr)
            MsgHandler::parsePath(window_, path);
        else
            ROS_INFO("Received message from /path, but no window is currently open.");
    }

    // Subscriber messages
    static nav_msgs::OccupancyGrid::ConstPtr map_;

    // Publisher
    static ros::Publisher pathreq_pub_;  // publisher for /pathreq topic
    static void publishPathRequest() {
        if (startX_ == -1 || startY_ == -1 || goalX_ == -1 || goalY_ == -1) {
            ROS_INFO("Both start and goal position must be set to publish PathRequest.");
            return;
        }
        // Create message
        nav_msgs::PathRequest pathreq;
        pathreq.start_x = (startX_ + 0.5f) * map_->gridsize;
        pathreq.start_y = (startY_ + 0.5f) * map_->gridsize;
        pathreq.goal_x = (goalX_ + 0.5f) * map_->gridsize;
        pathreq.goal_y = (goalY_ + 0.5f) * map_->gridsize;

        // Publish message
        pathreq_pub_.publish(pathreq);
        ROS_INFO("Published to /pathreq successfully.");

        // clear
        startX_ = startY_ = goalX_ = goalY_ = -1;
    }

    // Publisher message data
    static int startX_;
    static int startY_;
    static int goalX_;
    static int goalY_;

    static void cleanStartGoalPos() {
        if (startX_ != -1 && startY_ != -1) {
            PathRequestHandler::clearGrid(window_, startX_, startY_, map_->gridsize);
            // reset
            startX_ = -1;
            startY_ = -1;
        }
        if (goalX_ != -1 && goalY_ != -1) {
            PathRequestHandler::clearGrid(window_, goalX_, goalY_, map_->gridsize);
            // reset
            goalX_ = -1;
            goalY_ = -1;
        }
    }

    // Mouse Event Callback
    static void processMouseEvent(int event, int mouseX, int mouseY, int flags, void* param) {

        // Set start grid
        if (event == EVENT_LBUTTONDOWN) {  // left mouse click
            ROS_INFO("Left Mouse Click");
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
            // check button click events
            else {
                Button::processButtonClick(mouseX, mouseY);
            }
        }
        // Set goal grid
        else if (event == EVENT_RBUTTONDOWN) {  // right mouse click
            ROS_INFO("Right Mouse Click");
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
};

VisualizerWindow* VisualizerInterface::window_ = nullptr;
nav_msgs::OccupancyGrid::ConstPtr VisualizerInterface::map_ = nullptr;
ros::Publisher VisualizerInterface::pathreq_pub_;

int VisualizerInterface::startX_ = -1;
int VisualizerInterface::startY_ = -1;
int VisualizerInterface::goalX_ = -1;
int VisualizerInterface::goalY_ = -1;

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
