#ifndef PROV_CUBETRACK_H
#define PROV_CUBETRACK_H

#ifndef OPENCV
#define OPENCV 1
#endif
#ifndef TRACK_OPTFLOW
#define TRACK_OPTFLOW 1
#endif
#ifdef GPU
#undef GPU
#endif

#include <vector>
#include "../objects.hpp"
#include "../network/network.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/videoio.hpp>
#include "../network/target.hpp"

namespace ObjectTracking {

    class CubeTracker {
    private:
        Network& network;
    public:
        CubeTracker(Network& network);
        mutable std::mutex mutexTargets;
        std::vector<bbox_t> targetsLast;
        std::vector<bbox_t> targets;
        Tracker_optflow* tracker_flow;

        void update(std::vector<bbox_t>);
        void update(cv::Mat);
        void run(std::function<cv::Mat ()>);
    };

}

#endif
