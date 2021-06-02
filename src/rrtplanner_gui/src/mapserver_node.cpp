#include <math.h>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
//#include "lodepng/lodepng.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>  // for printing msg
#include <sstream>

using namespace cv;

class MapParser {
public:
    MapParser(ros::NodeHandle& nh) {
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
        parseMapData(img, img.cols, img.rows, gridSize, occupanyData);
        
        // publish map to ROS topic
        nav_msgs::OccupancyGrid og;
        og.name = filename;
        og.width = img.cols;
        og.height = img.rows;
        og.gridsize = gridSize;
        og.occupancy = occupanyData;
        map_pub_.publish(og);

        return true;  // parsed and published map to ROS topic successfully
    }

private:
    // Publisher
    ros::Publisher map_pub_;

    void parseMapData(const Mat& img, unsigned int width, unsigned int height, unsigned int gridSize, std::vector<unsigned char>& occupanyData) {

        ROS_INFO("Parsing map data ...");
        
        int cn = img.channels();
        std::cout << "No. channels: " << cn << std::endl;
        int threshold = ceil(gridSize*gridSize*0.5);    // threshold no. obstacle pixels to determine if a grid is an obstacle
        // iterate through grids
        for (int col = 0; col < width; col += gridSize) {
            for (int row = 0; row < height; row += gridSize) {
                // compute occupancy for each grid
                int obstacleCount = 0;  // no. obstacle pixels within this grid
                for (int i = row; i < row+gridSize; ++i) {
                    for (int j = col; j < col+gridSize; ++j) {
                        // compute color of pixel
                        int b = img.data[j*width*cn + i*cn];
                        int g = img.data[j*width*cn + i*cn + 1];
                        int r = img.data[j*width*cn + i*cn + 2];
                        int color = b + g + r;    // sum of pixel's RGB values
                        // check if pixel is an obstacle
                        if (color/3 > 200)  // obstacle pixel is white
                            ++obstacleCount;
                    }
                }
                // check if grid is occupied or not: obstacleCount > threshold, it is occupied
                unsigned char occupied = (obstacleCount >= threshold) ? 1 : 0;
                occupanyData.push_back(occupied);
                //	std::cout << (occupied ? 1 : 0);
            }
            //std::cout << std::endl;
        }
    }
};

int main(int argc, char* argv[]) {
    // check args first
    if (argc < 3) {
        ROS_INFO("Missing arguments");  // TODO: provide some info and instructions
        return 0;
    }
	
    // ROS init
    ros::init(argc, argv, "mapserver_node");
    ros::NodeHandle nh;

    // Create MapParser class for node's ROS interfaces and to parse a map image file
    MapParser mp(nh);

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