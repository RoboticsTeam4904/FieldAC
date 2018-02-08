#include "cubetrack.hpp"
#include <thread>

namespace ObjectTracking {
    CubeTracker::CubeTracker() = default;

    void CubeTracker::update(std::vector<Target> targetsUpdate) {
        this->mutexTargets.lock();
        this->targetsLast = this->targets;
        this->targets = targetsUpdate;
        this->mutexTargets.unlock();
    }

    void CubeTracker::run(std::function<cv::Mat ()> fetchFrame) {
        cv::Mat frame = fetchFrame();
        // TODO: C++ thrashed me and wouldn't let me check if the past targets were equal.
        // TODO: Hopefully I don't prank myself and someone else in the future fixes this
        // TODO: In all honestly it will probably be me though.
    }
}
