#include "cubetrack.hpp"
#include <thread>

namespace ObjectTracking {
    CubeTracker::CubeTracker() = default;

    void CubeTracker::update(std::vector<Target> targetsUpdate) {
        this->targetsLast = this->targets;
        this->targets = targetsUpdate;
    }

    void CubeTracker::run(std::function<cv::Mat ()> fetchFrame) {
        cv::Mat frame = fetchFrame();
        if(this->targets != this->targetsLast) {

        }
    }
}
