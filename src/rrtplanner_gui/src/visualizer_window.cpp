#include "visualizer_window.h"

VisualizerWindow::VisualizerWindow(const char* name, int w, int h) :
 _windowName((char*)name), _width(w), _height(h), _callback(NULL)
{
    // Create base Mat img
    _img = Mat::zeros(w, h, CV_8UC3);
}


void VisualizerWindow::setMouseCallbackFunc(MouseCallback callback) {
    _callback = callback;
    //setMouseCallback(_windowName, callback);
}

void VisualizerWindow::displayWindow() {
    // Show Mat in window
    imshow(_windowName, _img);
    if (_callback != NULL) {
        setMouseCallback(_windowName, _callback);
    }
    moveWindow(_windowName, 0, 200);
    waitKey(5);
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
