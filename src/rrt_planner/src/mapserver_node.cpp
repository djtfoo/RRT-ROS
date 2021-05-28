#include <ros/ros.h>

class MapParser {
public:
    MapParser(ros::NodeHandle& nh) {
        // TODO: sub/advertise options
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map");
    }

    void parse_map(char* filepath, int gridsize = 5) {
        // open path to map file

        // parse map image into grids
        std::vector<bool>& og;    // occupancy grid
        
        // publish map to ROS topic
        //map_pub_.publish(map);
    }

private:
    // Publisher
    ros::Publisher map_pub_;

    void parse_map_data(int gridsize, std::vector<bool>& og) {
        // 
    }
};

int main(int argc, char* argv[]) {
    // ROS init
    ros::init(argc, argv, "mapserver_node");
    ros::NodeHandle nh;

    // Create MapParser class for node's ROS interfaces and to parse a map image file
    MapParser mp(nh);

    // Parse map
//    mp.parse_map(argv[0],   // path to map image file
//	argv[1]    // size of a grid in pixels
//    );

    ROS_INFO("Map server node started");

    // Don't exit the program
    ros::spin();
}
