#include "vision.hpp"
#include <chrono>
#include <thread>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


namespace Vision {
    Camera::Camera(cv::String srcCapture) {
        this->devCapture = cv::VideoCapture();
        this->src = true;
        this->devCapture.open(srcCapture);
        if(!this->devCapture.isOpened()) {
            std::printf("--(!) Error opening source file\n");
            return;
        }
    }
    Camera::Camera(int devCapture) {
        this->devCapture = cv::VideoCapture();
        this->src = false;
        frameCount = 0;
        this->devCapture.open(devCapture);
        if(!this->devCapture.isOpened()) {
            std::printf("--(!) Error opening video capture\n");
            return;
        }
    }

    void Camera::registerListener(std::function<void (cv::Mat, int)> listener) {
        this->listenersMutex.lock();
        this->listeners.push_back(listener);
        this->listenersMutex.unlock();
    }

    void Camera::notifyListeners(cv::Mat update) {
        this->listenersMutex.lock();
        for(const auto &listener : this->listeners) {
            listener(update, frameCount);
        }
        this->listenersMutex.unlock();
    }

    void Camera::captureImages() {
        while (true) {
            frameMutex.lock();
            devCapture.read(frame);
            frameCount++;
            notifyListeners(getFrame());
            frameMutex.unlock();
            if(frame.empty()) {
                std::printf("The frame was empty here");
                return;
            }
            if(src) {
                // emulate camera delay
                std::this_thread::sleep_for(std::chrono::milliseconds(110));
            }
        }
    }

    bool Camera::displayImage(cv::Mat frame, const std::string window) {
        cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
        if(frame.empty()) {
            return false;
        }
        cv::imshow(window, frame);
        return (cv::waitKey(10) == 27);
    }

    cv::Mat Camera::getFrame() {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(), 0.5, 0.5, CV_INTER_LINEAR);
        return resized;
    }

    double Camera::getCapProp(int propId) {
        return this->devCapture.get(propId);
    }
}