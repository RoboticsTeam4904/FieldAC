#include <thread>
#include <opencv/cv.hpp>
#include "vision.hpp"
#include "objecttracking/cubetrack.hpp"

static const char* params =
        "{ help      | false | help                }"
        "{ dev       | 0     | camera device       }"
        "{ src       |       | source file         }"
        "{ cube_cfg  |       | cube model config   }"
        "{ cube_mdl  |       | cube model weights  }"
        "{ cube_cls  |       | cube class names    }"
        "{ cube_save |       | cube output file    }"
        "{ cube_cfd  | 0.5   | cube min confidence }";


int main(int argc, const char **argv) {
    cv::CommandLineParser parser(argc, argv, params);

    std::printf("Initializing camera...\n");
    Vision::Camera* defaultDev;
    if (parser.get<cv::String>("src").empty()) {
        defaultDev = new Vision::Camera(parser.get<int>("dev"));
    } else {
        defaultDev = new Vision::Camera(parser.get<cv::String>("src"));
    }
    std::printf("Beginning camera capture...\n");
    std::thread defaultDevCapture(&Vision::Camera::captureImages, defaultDev);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::printf("Initializing Object Tracking: Cube Tracker...\n");
    ObjectTracking::CubeTracker* cubeTracker;
    if(parser.get<cv::String>("cube_save").empty()) {
        cubeTracker = new ObjectTracking::CubeTracker(parser.get<cv::String>("cube_cfg"),
                                                  parser.get<cv::String>("cube_mdl"),
                                                  parser.get<cv::String>("cube_cls"));
    } else {
        cubeTracker = new ObjectTracking::CubeTracker(parser.get<cv::String>("cube_cfg"),
                                                  parser.get<cv::String>("cube_mdl"),
                                                  parser.get<cv::String>("cube_cls"),
                                                  parser.get<cv::String>("cube_src"),
                                                  defaultDev->getCapProp(cv::CAP_PROP_FRAME_WIDTH),
                                                  defaultDev->getCapProp(cv::CAP_PROP_FRAME_HEIGHT));
    }
    std::printf("Beginning cube tracking...\n");
    std::thread cubeTrackerRun(&ObjectTracking::CubeTracker::run, cubeTracker, std::bind(&Vision::Camera::getFrame, defaultDev));
    while(true) {
        if(defaultDev->displayImage(cubeTracker->getFrame(), "test display"))
            return -1;
    }
}