#ifndef VISUALIZER_WINDOW_H_
#define VISUALIZER_WINDOW_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

class VisualizerWindow {

    char _windowName[];  // name of window
    Mat _img;  // image to display in window
    int _width;  // width of window
    int _height; // height of window

public:
    VisualizerWindow(char* name, int w, int h);

    void createWindow();

    // draw shapes
    void MyEllipse(double angle);
    void MyFilledCircle(Point center);
    void MyPolygon();
    void MyLine(Point start, Point end);
};

#endif  // VISUALIZER_WINDOW_H_
