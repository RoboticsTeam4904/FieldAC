#ifndef VISION_H
#define VISION_H

#include <opencv2/core.hpp>

namespace Vision {
    extern cv::Mat frame;
    extern std::mutex frameMutex;

    void captureImages();
    void displayImage(cv::Mat* frame);

    cv::Mat* getFrame();
}

#endif
