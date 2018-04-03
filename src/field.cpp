// FieldAC
#include "vision.hpp"
#include "field.hpp"
// STD
#include <chrono>
#include <thread>
#include <sys/time.h>
#include <tuple>
#include <string>
#include <sstream>
#include <cstdarg>
#include <cmath>
#include <iostream>
#include <vector>
// OpenCV
#include <opencv/cv.hpp>
// WPISUITE
#include <ntcore.h>
#include <networktables/NetworkTable.h>

#define PI 3.14159265
#define NETWORKTABLES_PORT 1735
#define FT(CM) (CM * 0.0328084)
#define TEAM_NUMBER 4904
#define FOCAL_LENGTH 0.367
#define CUBE_SIZE 31.3
#define DEGRADATION_AMOUNT 0.05
#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))
#define ZRAND RAND -0.5


#define IMU_TO_CM_S2 980.6649999788 // Gs to cm/s^2

Field::Field() = default;

Field *Field::Field::instance = nullptr;

Field *Field::getInstance() {
    if (instance == nullptr) {
        instance = new Field();
    }
    return instance;
}

void Field::update(std::vector<bbox_t> cubeTargets) {
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

void Field::run() {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    while (true) {
        while (this->objects.empty() || !this->isReady) {
            continue;
        }
        this->old_data = latest_data;
//        this->get_sensor_data();
//        std::printf("got sensor data\n");
        // TODO not sure which accel is forward or lateral
        std::clock_t start = std::clock();
        bool lidarIsReady = true;
//        for (auto &p : pose_distribution) {
//            p.yaw = static_cast<float>(0);
//        }
//        this->scan_mutex.lock();
//        BotLocale::step(pose_distribution, old_data, latest_data, lidar_scans);
//        render();
//        this->scan_mutex.unlock();
//        int ms = (std::clock() - start) / (double) (CLOCKS_PER_SEC * 2.7 / 1000);
//        int fps = 1000 / (ms + 1);
//        std::cout << "Stepped in " << ms << "ms (" << fps << " hz)" << std::endl;
//        this->scan_mutex.lock();
//        me = BotLocale::get_best_pose(pose_distribution, this->lidar_scans.back());
//        this->scan_mutex.unlock();
//        me.yaw = static_cast<float>((latest_data.yaw + me.yaw) / 2);
//        std::printf("got best pose (%f, %f) at %f degrees moving (%f, %f) and turning %f\n", me.x, me.y,
//                    me.yaw * 180 / PI, me.dx, me.dy, me.rateYaw * 180 / PI);
    }
}

void Field::load() {
    /**
     *  TODO: Create a field from a vector graphic file-format
     *  TODO: using an std::vector of Segments
     */
    construct.emplace_back(Segment(75, 0, 0, 90));
    construct.emplace_back(Segment(0, 90, 0, 1572));
    construct.emplace_back(Segment(0, 1572, 75, 1660));
    construct.emplace_back(Segment(75, 1660, 474, 1660));
    construct.emplace_back(Segment(530, 1660, 775, 1660));
    construct.emplace_back(Segment(775, 1660, 850, 1572));
    construct.emplace_back(Segment(775, 1660, 850, 1572));
    construct.emplace_back(Segment(850, 90, 850, 1572));
    construct.emplace_back(Segment(775, 0, 850, 90));
    construct.emplace_back(Segment(75, 0, 320, 0));
    construct.emplace_back(Segment(376, 0, 775, 0));

    construct.emplace_back(Segment(212, 330, 212, 500));
    construct.emplace_back(Segment(212, 500, 636, 500));
    construct.emplace_back(Segment(212, 330, 636, 330));
    construct.emplace_back(Segment(636, 500, 636, 330));

    construct.emplace_back(Segment(290, 808, 402, 808));
    construct.emplace_back(Segment(402, 808, 402, 802));
    construct.emplace_back(Segment(402, 802, 448, 802));
    construct.emplace_back(Segment(448, 802, 448, 808));
    construct.emplace_back(Segment(448, 808, 560, 808));
    construct.emplace_back(Segment(560, 808, 560, 852));
    construct.emplace_back(Segment(402, 852, 402, 858));
    construct.emplace_back(Segment(402, 858, 448, 858));
    construct.emplace_back(Segment(448, 858, 448, 852));
    construct.emplace_back(Segment(448, 852, 560, 852));
    construct.emplace_back(Segment(402, 852, 290, 852));
    construct.emplace_back(Segment(290, 852, 290, 808));

    construct.emplace_back(Segment(212, 1159, 212, 1302));
    construct.emplace_back(Segment(212, 1302, 636, 1302));
    construct.emplace_back(Segment(212, 1159, 636, 1159));
    construct.emplace_back(Segment(636, 1302, 636, 1159));
//    construct.emplace_back(Segment(30, 30, 330, 30));
//    construct.emplace_back(Segment(330, 30, 330, 230));
//    construct.emplace_back(Segment(330, 230, 30, 230));
//    construct.emplace_back(Segment(30, 230, 30, 30));
    field_width = 0;
    field_height = 0;
    for (auto seg : this->construct) {
        if (std::get<0>(seg.start) > field_width) {
            field_width = std::get<0>(seg.start);
        }
        if (std::get<0>(seg.end) > field_width) {
            field_width = std::get<0>(seg.end);
        }
        if (std::get<1>(seg.start) > field_height) {
            field_height = std::get<1>(seg.start);
        }
        if (std::get<1>(seg.end) > field_height) {
            field_height = std::get<1>(seg.end);
        }
    }

    std::printf("Field generated.\n\tNumber of segments: %lu\n\tSize: %f x %f\n", construct.size(), field_height,
                field_width);
    me.x = 0;
    me.y = 0;
    me.yaw = 0; // forward/up
//    auto randomized = BotLocale::init();
//    for (int i = 0; i < SAMPLES; ++i) {
//        pose_distribution[i] = randomized[i];
//    }

    time_t seconds;
    time(&seconds);
    std::stringstream ss;
    ss << seconds;
    std::string ts = ss.str();
}

#pragma clang diagnostic pop