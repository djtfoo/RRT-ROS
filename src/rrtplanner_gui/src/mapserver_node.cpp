#include <math.h>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <iostream>  // for printing msg
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include "map/mapparser.h"

using namespace cv;

class MapServer {
public:
    MapServer(ros::NodeHandle& nh) {
        // TODO: sub/advertise options
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    }

    bool parseMap(std::string filename, unsigned int gridSize = 5) {

        //load map file using OpenCV
        Mat img = imread(filename, IMREAD_UNCHANGED);
        //img.convertTo(img, CV_8U);
        if (img.rows == 0 && img.rows == 0) {
            ROS_INFO("Failed to load file.");
            return false;
        }

        // TODO: check if image is divisible by gridsize?
        std::cout << "Image dimensions: " << img.rows << "," << img.cols << std::endl;

        // parse map image into grids
        std::vector<unsigned char> occupanyData;    // occupancy data of map grids
        MapParser::parseMapData(img, img.cols, img.rows, gridSize, occupanyData);
        
        // prepare ROS topic
        nav_msgs::OccupancyGrid og;
        og.name = filename;
        og.width = img.cols;
        og.height = img.rows;
        og.gridsize = gridSize;
        og.occupancy = occupanyData;
        // publish to ROS topic
        map_pub_.publish(og);

        return true;  // parsed and published map to ROS topic successfully
    }

private:
    // Publisher
    ros::Publisher map_pub_;
};

int main(int argc, char* argv[]) {
    // check args first
    if (argc <= 2) {
        ROS_INFO("Missing arguments");  // TODO: provide some info and instructions
        return 0;
    }
	
    // ROS init
    ros::init(argc, argv, "mapserver_node");
    ros::NodeHandle nh;

    // Create MapServer class for node's ROS interfaces and to parse a map image file
    MapServer mp(nh);

    ROS_INFO("Map server node started");
    ros::Duration(1).sleep();  // sleep for 1 second

    // Parse map image file argument
    std::string mf(argv[1]);  // map image file
    // Parse grid size argument
    std::stringstream ss;
    ss << argv[2];
    unsigned int gs;  // grid size
    ss >> gs;

    std::cout << "Map file: " << argv[1] << std::endl;
    std::cout << "Grid size (pixels): " << gs << std::endl;

    // Parse map
    bool success = mp.parseMap(
        mf,   // path to map image file
        gs    // size of a grid in pixels
    );

    if (success)
        ROS_INFO("Map occupancy grid published to /map successfully.");

    // Don't exit the program
    //ros::spin();

    return 0;
}
