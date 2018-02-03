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
        mutable std::mutex mutexTargets;
        std::vector<Target> targetsLast;
        std::vector<Target> targets;

        void update(std::vector<Target>);

        void run(std::function<cv::Mat ()>);
    };

}

#endif
