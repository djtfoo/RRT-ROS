#ifndef VISUALIZER_WINDOW_H_
#define VISUALIZER_WINDOW_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;

class VisualizerWindow {

    char* _windowName;  // name of window
    Mat _img;  // image to display in window
    int _width;  // width of window
    int _height; // height of window

    MouseCallback _callback;  // mouse callback function

public:
    // init
    VisualizerWindow(const char* name, int w, int h);
    void setMouseCallbackFunc(MouseCallback callback);

    void displayWindow();

    // draw shapes
    void drawCircle(const Point& center, int radius, const Scalar& color, int thickness);
    void drawRectangle(const Point& min, const Point& max, const Scalar& color, int thickness);
    void drawLine(const Point& start, const Point& end, const Scalar& color, int thickness);
};

#endif  // VISUALIZER_WINDOW_H_
