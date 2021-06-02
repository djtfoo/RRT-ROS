#include <ros/ros.h>
#include "rrt_planner.h"
#include "rrtstar_planner.h"

enum RrtVersion {
    Basic_RRT,  // 0
    RRT_Star,   // 1
};

class RrtPlannerInterface {
public:
    // Constructor
    RrtPlannerInterface(ros::NodeHandle* nh) {
        // subscribe to ROS topics
        map_sub_ = nh->subscribe<nav_msgs::OccupancyGrid>("map", 1, mapCallback);
        pathreq_sub_ = nh->subscribe<nav_msgs::PathRequest>("pathreq", 1, pathreqCallback);

        // store reference to NodeHandle
        nh_ = nh;
    }

private:
    static ros::NodeHandle* nh_;

    // Subscriber
    ros::Subscriber map_sub_;
    ros::Subscriber pathreq_sub_;

    // Subscriber messages
    static nav_msgs::OccupancyGrid::ConstPtr map_;

    // Subscriber callback
    static void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        ROS_INFO("Map message received");

        // Save map
        map_ = map;
    }

    static void pathreqCallback(const nav_msgs::PathRequest::ConstPtr& pathreq) {
        ROS_INFO("Path Request message received");

        // Set start and end positions
        Coord start(pathreq->start_x, pathreq->start_y);
        Coord goal(pathreq->goal_x, pathreq->goal_y);

        // Validate start and end coordinates first (command line publisher did not validate them)
        int gridWidth = map_->width / map_->gridsize;
        bool isValid = true;

        int startX = start._x / map_->gridsize;
        int startY = start._y / map_->gridsize;
        if (map_->occupancy[startY*gridWidth + startX]) {
            ROS_INFO("Start position is in an obstacle");
            isValid = false;
        }
        int goalX = goal._x / map_->gridsize;
        int goalY = goal._y / map_->gridsize;
        if (map_->occupancy[goalY*gridWidth + goalX]) {
            ROS_INFO("Goal position is in an obstacle");
            isValid = false;
        }

        // Plan path
        if (isValid) {
            ROS_INFO("Starting RRT.");
            // Create RRT Planner
            RrtPlanner* rrtPlanner;
            switch (pathreq->rrt_ver) {
            case Basic_RRT: {
                RrtPlanner rrtPlanner(*nh_);
                rrtPlanner.planPath(map_, start, goal);
            }
                break;
            case RRT_Star: {
                RrtStarPlanner rrtPlanner(*nh_);
                rrtPlanner.planPath(map_, start, goal);
            }
                break;
            }
        }
        else
            ROS_INFO("Not starting RRT as start/goal position is invalid.");
    }
};

ros::NodeHandle* RrtPlannerInterface::nh_ = nullptr;
nav_msgs::OccupancyGrid::ConstPtr RrtPlannerInterface::map_;

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;

    // Create Planner class for node's ROS interfaces and path planning
    RrtPlannerInterface rrtPlannerInterface(&nh);

    ROS_INFO("Planner node started");

    // TODO: use argv to enable/disable visualization

    // Don't exit the program
    ros::spin();
}
