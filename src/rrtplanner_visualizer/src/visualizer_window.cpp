#include "visualizer_window.h"

VisualizerWindow::VisualizerWindow(char* name, int w, int h) :
 _windowName(name), _width(w), _height(h)
{
    // Create base Mat img
    _img = Mat::zeros(w, h, CV_8UC3);
}

void VisualizerWindow::createWindow() {
    // Destroy existing window if any?

    // Show Mat in window
    imshow(_windowName, _img);
}
