#include "visualizer_window.h"
#include "button.h"

VisualizerWindow::VisualizerWindow(const char* name, int w, int h) :
 _windowName((char*)name), _width(w), _height(h), _callback(NULL)
{
    // Create base Mat img
    _img = Mat::zeros(h+_bottomBarHeight, w, CV_8UC3);

    // Create bottom bar
    drawRectangle(Point(0, h), Point(w, h+_bottomBarHeight), Scalar(100, 100, 100), -1);

    // Draw Buttons
    Button::drawButtons(w, h+_bottomBarHeight, this);
}

VisualizerWindow::~VisualizerWindow() {
    // Destroy window
    cvDestroyWindow(_windowName);
}

void VisualizerWindow::setMouseCallbackFunc(MouseCallback callback) {
    _callback = callback;
    setMouseCallback(_windowName, callback);
}

void VisualizerWindow::displayWindow() {
    // Show Mat in window
    imshow(_windowName, _img);
    if (_callback != NULL) {
        setMouseCallback(_windowName, _callback);
    }
    moveWindow(_windowName, 0, 200);
    waitKey(2);
}

/* Drawer functions */
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

void VisualizerWindow::putText(const char* text, const Point& topleftPos, float fontSize, const Scalar& color, int thickness) {
    cv::putText(_img,
        text,
        Point(topleftPos.x, topleftPos.y),
        FONT_HERSHEY_DUPLEX,
        fontSize,
        color,
        thickness
    );
}
