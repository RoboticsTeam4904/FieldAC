#ifndef CUBETRACK_H
#define CUBETRACK_H

#include <vector>
#include "../objects.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/videoio.hpp>

namespace ObjectTracking {

    class CubeTracker {
    public:
        CubeTracker();

        CubeTracker(cv::String config, cv::String model, cv::String classNames);
        CubeTracker(cv::String config, cv::String model, cv::String classNames, cv::String save, double capWidth, double capHeight);
        cv::dnn::Net darknet;
        std::vector<cv::String> classNames;
        cv::VideoWriter saveWriter;

        cv::Mat frame;
        mutable std::mutex frameMutex;

        void run(std::function<cv::Mat ()> frameFunc);
        cv::Mat getFrame();

        /**
         * Overloading the `=` operator.
         * We ensure that the copying of members happens in a locked state for
         * for both sides of the assignment, and more importantly that the locking
         * is safely released after the assignment is completed.
         */
        CubeTracker& operator=(const CubeTracker& origin) {
            if (this != &origin) {
                std::lock(frameMutex, origin.frameMutex);
                std::lock_guard<std::mutex> lhs_lk(frameMutex, std::adopt_lock);
                std::lock_guard<std::mutex> rhs_lk(origin.frameMutex, std::adopt_lock);
                darknet = origin.darknet;
                classNames = origin.classNames;
                saveWriter = origin.saveWriter;

                frame = origin.frame;
            }
            return *this;
        }
    };


    void tick();
}

#endif
