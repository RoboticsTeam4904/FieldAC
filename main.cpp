#include <thread>
#include <opencv/cv.hpp>
#include <unordered_map>
#include "vision.hpp"
#include "objecttracking/cubetrack.hpp"
#include "network/network.hpp"

static const char* params =
        "{ help     | false | help                }"
        "{ dev      | 0     | camera device       }"
        "{ src      |       | source file         }"
        "{ net_cls  |       | net class names     }"
        "{ net_cfg  |       | net model config    }"
        "{ net_mdl  |       | net model weights   }"
        "{ net_save |       | net output file     }"
        "{ net_cfd  | 0.5   | net min confidence  }";


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

    std::printf("Initializing Darknet...");
    Network* network;
    if(parser.get<cv::String>("net_save").empty()) {
        network = new Network(parser.get<cv::String>("net_cls"),
                              parser.get<cv::String>("net_cfg"),
                              parser.get<cv::String>("net_mdl"));
    } else {
        network = new Network(parser.get<cv::String>("net_cls"),
                              parser.get<cv::String>("net_cfg"),
                              parser.get<cv::String>("net_mdl"),
                              parser.get<cv::String>("net_save"),
                              defaultDev->getCapProp(cv::CAP_PROP_FRAME_WIDTH),
                              defaultDev->getCapProp(cv::CAP_PROP_FRAME_HEIGHT));
    }
    network->run([defaultDev]() {
        return defaultDev->getFrame();
    },
                 {
                         {"cube", [cubeTracker](cv::Mat frame, std::vector<Target> targets) {
                             return cubeTracker->run(frame, targets);
                         }}
                 }
    );
//    std::thread networkRun(&Network::run,
//                           network,
//                           [defaultDev]() {
//                               return defaultDev->getFrame();
//                           }, std::unordered_map<std::string, std::function<void(cv::Mat, std::vector<Target>)>>
//                                   {{"person", [cubeTracker](cv::Mat frame, std::vector<Target> targets) {
//                                       return cubeTracker->run(frame, targets);
//                                   }}}
//    );
//    while(true) {
//        if(defaultDev->displayImage(network->getAnnotatedFrame(), "Darknet")) {
//            return -1;
//        }
//    }
}