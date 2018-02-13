#ifndef PROV_VISION_H
#define PROV_VISION_H

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

        /**
         * Overloading the `=` operator.
         * We ensure that the copying of members happens in a locked state for
         * for both sides of the assignment, and more importantly that the locking
         * is safely released after the assignment is completed. .
         */
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
