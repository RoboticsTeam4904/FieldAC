#ifndef CUBETRACK_H
#define CUBETRACK_H

#include <vector>
#include "../objects.hpp"
//#include "../darknet/network.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/videoio.hpp>

namespace ObjectTracking {

    class CubeTracker {
    public:
        CubeTracker();

        void run(cv::Mat frame, std::vector<Pose> targets);
    };

}

#endif
