#include "visualizer_window.h"

#include <ros/ros.h>

VisualizerWindow::VisualizerWindow(const char* name, int w, int h) :
 _windowName((char*)name), _width(w), _height(h)
{
    // Create base Mat img
    _img = Mat::zeros(w, h, CV_8UC3);
}

void VisualizerWindow::displayWindow() {
    // Show Mat in window
    imshow(_windowName, _img);
    moveWindow(_windowName, 0, 200);
    waitKey(10);
}

void VisualizerWindow::drawCircle(const Point& center, int radius, const Scalar& color, int thickness) {
    circle(_img,
        center,
        radius,
        color,
        thickness,
        LINE_8
    );
}

void VisualizerWindow::drawRectangle(const Point& min, const Point& max, const Scalar& color, int thickness)
{
    rectangle(_img,
        min,
        max,
        color,
        thickness,
        LINE_8
    );
}
void VisualizerWindow::drawLine(const Point& start, const Point& end, const Scalar& color, int thickness)
{
    line(_img,
        start,
        end,
        color,
        thickness,
        LINE_8
    );
}
