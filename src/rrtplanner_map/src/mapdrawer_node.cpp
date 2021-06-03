#include <ros/ros.h>
#include <thread>
#include <sstream>
#include <math.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include "gui/visualizer_window.h"
#include "gui/button.h"
#include "msg_handler.h"
#include "pathrequest/pathrequest_handler.h"
#include "map/mapparser.h"

using namespace cv;

class MapDrawerInterface {
public:
    MapDrawerInterface(ros::NodeHandle& nh)
    {
        // Advertise ROS topics
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);

        // add Button
        Button::createButton(Anchor(BottomLeft, 50, 5), 120, 30, 180, 180, 180, "Publish Map", (void*)publishMapCallback);
        Button::createButton(Anchor(BottomLeft, 200, 5), 120, 30, 180, 180, 180, "Export Map", (void*)exportMapCallback);

        // Create window
        window_ = new VisualizerWindow("Custom", width, height);
        window_->setMouseCallbackFunc(processMouseEvent);
    }
    ~MapDrawerInterface() {
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

    // Map variables
    static void setWidth(int w) {
        width = w;
    }
    static void setHeight(int h) {
        height = h;
    }
    static void setGridSize(int gs) {
        gridSize = gs;
    }

private:
    // Map variables
    static int width;
    static int height;
    static int gridSize;

    // window
    static VisualizerWindow* window_;  // only 1 window

    // Publisher
    static ros::Publisher map_pub_;  // publisher for /map topic
    static void publishMapCallback() {
        Mat img = (window_->getImage());
        // Create map
        std::vector<unsigned char> occupanyData;    // occupancy data of map grids
        MapParser::parseMapData(img, width, height, gridSize, occupanyData);
        
        // Prepare ROS topic
        nav_msgs::OccupancyGrid og;
        og.name = "maps/custom";
        og.width = width;
        og.height = height;
        og.gridsize = gridSize;
        og.occupancy = occupanyData;
        // Publish to ROS topic
        map_pub_.publish(og);

        ROS_INFO("Published to /map successfully.");

        // close window and exit
        ros::shutdown();
    }

    // Mouse Event Callback
    static int mouseButtonDownEvent;  // which button was pressed
    static void processMouseEvent(int event, int mouseX, int mouseY, int flags, void* param) {

        if (event == EVENT_MOUSEMOVE) {  // mouse moved, continue previous mouse action
            event = mouseButtonDownEvent;
        }

        // Set start grid
        if (event == EVENT_LBUTTONDOWN) {  // left mouse click
            ROS_INFO("Left Mouse Click");
            // check if grid is valid
            if (mouseX >= 0 && mouseX < width && mouseY >= 0 && mouseY < height) {  // grid is within map bounds
                // paint (7 pixels wide)
                window_->drawRectangle(Point(fmax(mouseX-3, 0), fmax(mouseY-3, 0)), Point(fmin(mouseX+3, width-1), fmin(mouseY+3, height-1)), Scalar(255, 255, 255), -1);
            }
            // check button click events
            else {
                Button::processButtonClick(mouseX, mouseY);
            }
        }
        else if (event == EVENT_RBUTTONDOWN) {  // right mouse click
            ROS_INFO("Right Mouse Click");
            // check if grid is valid
            if (mouseX >= 0 && mouseX < width && mouseY >= 0 && mouseY < height) {  // grid is within map bounds
                // eraser (15 pixels wide)
                window_->drawRectangle(Point(fmax(mouseX-7, 0), fmax(mouseY-7, 0)), Point(fmin(mouseX+7, width-1), fmin(mouseY+7, height-1)), Scalar(0, 0, 0), -1);
            }
            // check button click events
            else {
                Button::processButtonClick(mouseX, mouseY);
            }
        }
        // store the mouse event
        mouseButtonDownEvent = event;
    }

    // Export map image
    static void exportMapCallback() {
        // Crop custom map image from window
        Mat cropped;
        Rect myROI(0, 0, width, height);  // region of interest
        Mat croppedRef(window_->getImage(), myROI);  // crop the full image to that image contained by the rectangle myROI
        croppedRef.copyTo(cropped);  // copy the data into new matrix

        // Save as custom.jpg
        if (imwrite("maps/custom.jpg", cropped))
            ROS_INFO("Map exported successfully.");
        else
            ROS_INFO("An error occurred when exporting the image.");

        // Close window and exit
        ros::shutdown();
    }
};

VisualizerWindow* MapDrawerInterface::window_ = nullptr;
ros::Publisher MapDrawerInterface::map_pub_;
int MapDrawerInterface::width = 500;
int MapDrawerInterface::height = 500;
int MapDrawerInterface::gridSize = 5;
int MapDrawerInterface::mouseButtonDownEvent;

int main(int argc, char* argv[]) {
    std::stringstream ss;
    // parse arguments for map width, height and gridsize
    if (argc >= 2) { // width parameter
        int w;
        ss << argv[1];
        ss >> w;
        MapDrawerInterface::setWidth(w);
    }
    if (argc >= 3) { // height parameter
        ss.clear();
        int h;
        ss << argv[2];
        ss >> h;
        MapDrawerInterface::setHeight(h);
    }
    if (argc >= 4) { // gridSize parameter
        ss.clear();
        int gs;
        ss << argv[3];
        ss >> gs;
        MapDrawerInterface::setGridSize(gs);
    }

    // ROS init
    ros::init(argc, argv, "mapdrawer_node");
    ros::NodeHandle nh;

    // Create Visualizer node's ROS interfaces
    MapDrawerInterface mapDrawerInterface(nh);

    ROS_INFO("Map Drawer node started");

    // Create visualizer thread
    std::thread th1(mapDrawerInterface.refreshWindow);

    // Don't exit the program
    ros::spin();
}
