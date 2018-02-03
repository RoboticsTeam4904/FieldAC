#include "cubetrack.hpp"
#include <thread>
#include <utility>

namespace ObjectTracking {
    CubeTracker::CubeTracker() = default;

    void CubeTracker::update(std::vector<Target> targetsUpdate) {
        this->targetsLast = std::move(this->targets);
        this->targets = std::move(targetsUpdate);
    }

    void CubeTracker::run(std::function<cv::Mat ()> fetchFrame) {
        auto frame := fetchFrame();
        if(this->targets != this->targetsLast) {

        }
    }

    void CubeTracker::run(std::function<std::vector<Target>(std::string object)> targetFetch) {
        std::vector<Target> targets = targetFetch("cube");
        if(targets != this->lastTargets) {

        }
    }

}
