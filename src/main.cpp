#define OPENCV 1
#define TRACK_OPTFLOW 1
#undef GPU

#include <thread>
#include <opencv/cv.hpp>
#include <unordered_map>
#include "vision.hpp"
#include "network/network.hpp"
#include "objecttracking/cubetrack.hpp"
#include "botlocale/lidar.hpp"
#include "field.hpp"

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

static const char* about =
        "FRC Field Model by Bot Provoking (https://github.com/roboticsteam4904/FieldAC)\n"
        "Constructs a real time model of the field and all its dynamic constituent pieces\n"
        "such as other robots; small yearly game pieces like cubes, gears, or balls;\n"
        "and provides functionality for localization within the model.\n\n";

static const char* params =
        "{ help     | false  | help                                                            }"
        "{ dev      | 0      | Capture Device                                                  }"
        "{ src      |        | Source Video file. Overrides any specified capture device       }"
        "{ net_cls  |        | [Network] \".names\" file for identifiable classes              }"
        "{ net_cfg  |        | [Network] Model \".cfg\" file                                   }"
        "{ net_mdl  |        | [Network] Model \".weights\" file                               }"
        "{ net_save |        | [Network] Detection output file. Shows what the network detects }"
        "{ net_cfd  | 0.5    | [Network] Minimum identification confidence threshold           }"
        "{ ldr_dev  |        | [LIDAR] Path to the *nix device (eg. /dev/ttyUSB0)              }"
        "{ ldr_baud | 115200 | [LIDAR] Baudrate for serial communications                      }";

int main(int argc, const char **argv) {
    cv::CommandLineParser parser(argc, argv, params);

    if (parser.get<bool>("help")) {
        // Compiler advises to treat string like so.
        std::printf("%s", about);
        parser.printMessage();
        return 0;
    }

//    std::printf("Initializing Robot comms...");
//    Socket* socket = new Socket("127.0.0.1", 5021);

    std::printf("Initializing Camera...\n");
    Vision::Camera* defaultDev;
    if (parser.get<cv::String>("src").empty()) {
        defaultDev = new Vision::Camera(parser.get<int>("dev"));
    } else {
        defaultDev = new Vision::Camera(parser.get<cv::String>("src"));
    }

    std::printf("Initializing Darknet...");
    Network* network;

    std::printf("Beginning camera capture...\n");
    std::thread defaultDevCapture(&Vision::Camera::captureImages, defaultDev);
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

    std::printf("Initializing Object Tracking: Cube Tracker...\n");
    auto cubeTracker = new ObjectTracking::CubeTracker(*network);

    std::printf("Registering camera listener: Cube Tracker...\n");
    defaultDev->registerListener([cubeTracker](cv::Mat mat, int frameCount) {
        cubeTracker->update(mat, frameCount);
    });
    std::printf("Registering camera listener: Network...\n");
    defaultDev->registerListener([network](cv::Mat mat, int frameCount) {
        network->update(mat, frameCount);
    });

    std::printf("Initializing Lidar...\n");
    Lidar* lidar = new Lidar(parser.get<cv::String>("ldr_dev"),
                             parser.get<uint32_t>("ldr_baud"));

//    This code is non-threaded but also serves as a
//    slightly cleaner demonstration of what's really being run.
//
//    network->run([defaultDev]() {
//        return defaultDev->getFrame();
//    },
//                 {
//                         {"cube", [cubeTracker](std::vector<Target> targets) {
//                             return cubeTracker->update(targets);
//                         }}
//                 }
//    );
//
//    std::thread networkRun(&Network::run,
//                           network,
//                           [defaultDev]() {
//                               return defaultDev->getFrame();
//                           }, std::unordered_map<std::string, std::function<void(std::vector<bbox_t>)>>
//                           {
//                                   {"cube", [cubeTracker](std::vector<bbox_t> targets) {
//                                       return cubeTracker->update(targets);
//                                   }}}
//    );
//
//    std::thread cubetrackRun(&ObjectTracking::CubeTracker::run,
//                             cubeTracker);
    std::printf("\n");

    std::thread lidarRun(&Lidar::run,
                         lidar,
                         &ctrl_c_pressed);

    Field* field = Field::getInstance();
    field->load();
    while(true) {
        if(defaultDev->displayImage(cubeTracker->optflowFrame, "Optflow")) {
            return -1;
        }
        if(cv::waitKey(10) == 32) {
            cubeTracker->recalc = true;
        }
        if(defaultDev->displayImage(network->getAnnotatedFrame(), "Darknet")) {
            return -1;
        }
        if (ctrl_c_pressed){
            break;
        }
        field->update(cubeTracker->optflow_targets);
        field->update(lidar->current_scan);
        field->tick();
        defaultDev->displayImage(field->renderedImage, "Field");
    }
}
