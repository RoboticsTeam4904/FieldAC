#ifndef CUBETRACK_H
#define CUBETRACK_H

#include <vector>
#include "../objects.hpp"
#include "../network/network.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/videoio.hpp>

namespace ObjectTracking {

    class CubeTracker {
    public:
        CubeTracker();
        std::vector<Target> lastTargets;

        void run(cv::Mat frame, std::vector<Target> targets);
        void run(std::function<std::vector<Target> (std::string object)>);
    };

}

#endif
