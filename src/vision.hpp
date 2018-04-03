#ifndef PROV_VISION_H
#define PROV_VISION_H

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <queue>
#include <mutex>

namespace Vision {
    class Camera {
    public:
        static constexpr float FOCAL_LENGTH = 0.367;
    private:
        mutable std::mutex frameMutex;
        mutable std::mutex listenersMutex;
        std::vector<std::function<void (cv::Mat, int)>> listeners;
        bool src;
    public:
        explicit Camera(int devCapture);
        explicit Camera(cv::String srcCapture);

        cv::Mat frame;
        cv::VideoCapture devCapture;

        void captureImages();
        bool displayImage(cv::Mat frame, const std::string window);
        void registerListener(std::function<void (cv::Mat, int)> listener);

        // These are outdated, consider deprecating/removing.
        double getCapProp(int propId);
        cv::Mat getFrame();

        /**
         * Overloading the `=` operator.
         * We ensure that the copying of members happens in a locked state for
         * for both sides of the assignment, and more importantly that the locking
         * is safely released after the assignment is completed. .
         */
        Camera& operator=(const Camera& origin) {
            if (this != &origin) {
                std::lock(frameMutex, origin.frameMutex);
                std::lock_guard<std::mutex> frm_lhs_lk(frameMutex, std::adopt_lock);
                std::lock_guard<std::mutex> frm_rhs_lk(origin.frameMutex, std::adopt_lock);
                std::lock(listenersMutex, origin.listenersMutex);
                std::lock_guard<std::mutex> lsr_lhs_lk(listenersMutex, std::adopt_lock);
                std::lock_guard<std::mutex> lsr_rhs_lk(origin.listenersMutex, std::adopt_lock);
                frame = origin.frame;
                listeners = origin.listeners;
                devCapture = origin.devCapture;
            }
            return *this;
        }
    private:
        int frameCount;
        void notifyListeners(cv::Mat update);
    };
    std::tuple<float, float> pixel_to_rad(float x, float y, double fov_degrees, int w, int h);
}

#endif
