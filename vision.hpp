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
    bool displayImage(cv::Mat* frame, const std::string window);

    cv::Mat* getFrame();
}

#endif
