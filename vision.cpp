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
//            std::printf("Are we getting out of here?\n");
            if(frame.empty()) {
                std::printf("The frame was empty here");
                return;
            }
        }
    }

    bool Camera::displayImage(cv::Mat frame, const std::string window) {
        cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
//        std::printf("%d\n", annotatedFrame.cols == 0);
        if(frame.empty()) {
            return false;
        }
        cv::imshow(window, frame);
        return (cv::waitKey(10) == 27 );
    }

    cv::Mat Camera::getFrame() {
//        frameMutex.lock();
//        devCapture.read(frame);
//        frameMutex.unlock();
        return frame;
    }

    double Camera::getCapProp(int propId) {
        return this->devCapture.get(propId);
    }
}