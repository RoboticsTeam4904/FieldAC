#ifndef VISION_H
#define VISION_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

namespace Vision {
    class Camera {
    public:
        explicit Camera(int devCapture);
        explicit Camera(cv::String srcCapture);

        cv::Mat frame;
        mutable std::mutex frameMutex;
        cv::VideoCapture devCapture;

        void captureImages();
        bool displayImage(cv::Mat frame, const std::string window);
        cv::Mat getFrame();

        double getCapProp(int propId);

        Camera& operator=(const Camera& origin) {
            if (this != &origin) {
                std::lock(frameMutex, origin.frameMutex);
                std::lock_guard<std::mutex> lhs_lk(frameMutex, std::adopt_lock);
                std::lock_guard<std::mutex> rhs_lk(origin.frameMutex, std::adopt_lock);
                frame = origin.frame;
                devCapture = origin.devCapture;
            }
            return *this;
        }
    };
}

#endif
