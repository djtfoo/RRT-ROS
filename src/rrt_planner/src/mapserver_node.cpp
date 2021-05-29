#include <math.h>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include "lodepng/lodepng.h"

#include <iostream>  // for logging
#include <sstream>

class MapParser {
public:
    MapParser(ros::NodeHandle& nh) {
        // TODO: sub/advertise options
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    }

    bool parseMap(std::string filename, unsigned int gridSize = 5) {
        std::vector<unsigned char> png;
        std::vector<unsigned char> image; //the raw pixels
        unsigned width, height;

        //load and decode map file (for now, we only accept PNG files)
        unsigned error = lodepng::load_file(png, filename);
        if (error) {
            ROS_INFO("Error when loading map image file");
            //ROS_INFO(lodepng_error_text(error));
            return false;
        }
        error = lodepng::decode(image, width, height, png);
        if (error) {
            ROS_INFO("Error when decoding map image");
            //ROS_INFO(lodepng_error_text(error));
            return false;
        }

        // TODO: check if image is divisible by gridsize?
        std::cout << width << "," << height << std::endl;

        // parse map image into grids
        std::vector<unsigned char> occupanyData;    // occupancy data of map grids
        parseMapData(image, width, height, gridSize, occupanyData);
        
        // publish map to ROS topic
        nav_msgs::OccupancyGrid og;
        og.width = width;
        og.height = height;
        og.gridsize = gridSize;
        og.occupancy = occupanyData;
        map_pub_.publish(og);

        return true;  // parsed and published map to ROS topic successfully
    }

private:
    // Publisher
    ros::Publisher map_pub_;

    void parseMapData(const std::vector<unsigned char>& image, unsigned int width, unsigned int height, unsigned int gridSize, std::vector<unsigned char>& occupanyData) {

        ROS_INFO("Parsing map data ...");
        
        int threshold = ceil(gridSize*gridSize*0.5);    // threshold for whether a grid is an obstacle
        
        // iterate through grids
        for (int row = 0; row < height; row += gridSize) {
            for (int col = 0; col < width; col += gridSize) {
                // compute occupancy for each grid
                int obstacleCount = 0;  // no. obstacle pixels within this grid
                for (int i = row; i < row+gridSize; ++i) {
                    for (int j = col; j < col+gridSize; ++j) {
                        // compute color of pixel
                        int pixelIdx = 4*(i*height + j);  // index of pixel's RGBA values
                        int color = image[pixelIdx] + image[pixelIdx+1] + image[pixelIdx+2];    // sum of pixel's RGB values
                        // check if pixel is an obstacle
                        if (color/3 > 200)  // obstacle pixel is white
                            ++obstacleCount;
                    }
                }
                // check if grid is occupied or not: obstacleCount > threshold, it is occupied
                unsigned char occupied = (obstacleCount >= threshold) ? 1 : 0;
                occupanyData.push_back(occupied);
                std::cout << (occupied ? 1 : 0);
            }
            std::cout << std::endl;
        }
    }
};

int main(int argc, char* argv[]) {
    // check args first
    if (argc < 2) {
        ROS_INFO("Missing arguments");
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

    std::cout << "Map file: " << argv[2] << std::endl;
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
