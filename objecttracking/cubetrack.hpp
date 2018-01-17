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

        void run(cv::Mat *frame);
    };


    void tick();
}

#endif
