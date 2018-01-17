#include "vision.hpp"

#include <opencv2/highgui.hpp>

namespace Vision {
    Camera::Camera(cv::String srcCapture) {
        this->devCapture = cv::VideoCapture();
        this->devCapture.open(srcCapture);
        if(!this->devCapture.isOpened()) {
            std::printf("--(!) Error opening source file\n");
            return;
        }
    }
    Camera::Camera(int devCapture) {
        this->devCapture = cv::VideoCapture();
        this->devCapture.open(devCapture);
        if(!this->devCapture.isOpened()) {
            std::printf("--(!) Error opening video capture\n");
            return;
        }
    }

    void Camera::captureImages() {
        while (true) {
            frameMutex.lock();
            devCapture.read(frame);
            frameMutex.unlock();
            if(frame.empty()) {
                return;
            }
        }
    }

    bool Camera::displayImage(cv::Mat* frame, const std::string window) {
        cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
        if(frame->empty()) {
            return false;
        }
        cv::imshow(window, *frame);
        return (cv::waitKey(10) == 27 );
    }

    cv::Mat* Camera::getFrame() {
        return &frame;
    }

    double Camera::getCapProp(int propId) {
        return devCapture.get(propId);
    }
}