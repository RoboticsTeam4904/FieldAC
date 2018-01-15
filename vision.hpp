#ifndef VISION_H
#define VISION_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace Vision {
    extern cv::Mat frame;
    extern std::mutex frameMutex;
    extern cv::VideoCapture devCapture;

    void init();
    void captureImages();
    bool displayImage();

    cv::Mat* getFrame();
}

#endif
