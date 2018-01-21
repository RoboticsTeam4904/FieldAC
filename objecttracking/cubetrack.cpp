#include "cubetrack.hpp"
//#include "../darknet/network.hpp"

#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>
#include <fstream>
#include <thread>

namespace ObjectTracking {
    CubeTracker::CubeTracker() = default;

    void CubeTracker::run(cv::Mat frame, std::vector<Pose> targets) {
        for(const auto &target : targets) {
//            std::printf("Found target: xCenter: %f.2, yCenter: %f.2, width: %f.2, height: %f.2, confidence: %f.2\n",
//            target.xCenter, target.yCenter, target.width, target.height, target.confidence);
        }
    }

}
