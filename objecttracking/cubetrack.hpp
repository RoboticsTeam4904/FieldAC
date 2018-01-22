#ifndef CUBETRACK_H
#define CUBETRACK_H

#include <vector>
#include "../network/network.hpp"
#include <opencv2/videoio.hpp>

namespace ObjectTracking {

    class CubeTracker {
    public:
        CubeTracker();

        void run(cv::Mat frame, std::vector<Target> targets);
    };

}

#endif
