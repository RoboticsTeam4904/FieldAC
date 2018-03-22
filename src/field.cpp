#include "field.hpp"
#include "./network/network.hpp"
#include <chrono>
#include <thread>
#include <sys/time.h>
#include <math.h>
#include <tuple>
#include "./botlocale/lidar.hpp"

#define PI 3.14159265
#define NETWORKTABLES_PORT 12345 // i don't know what it actually is
#define TEAM_NUMBER 4904
#define NACHI_SUQQQQ 1

Segment::Segment(int xi, int yi, int xf, int yf) {
    this->start = std::tuple<int, int>(xi, yi);
    this->end = std::tuple<int, int>(xf, yf);
}

Segment::Segment(std::tuple<int, int> start, std::tuple<int, int> end) {
    this->start = start;
    this->end = end;
}

Field::Field() = default;

Field *Field::Field::instance = nullptr;

Field *Field::getInstance() {
    if (instance == nullptr) {
        instance = new Field();
    }
    return instance;
}

void Field::load() {
    /**
     *  TODO: Create a field from a vector graphic file-format
     *  TODO: using an std::vector of Segments
     */
    me.x = 250;
    me.y = 250;
    me.yaw = 0; // forward/up
}

void Field::update(std::vector<bbox_t> cubeTargets) {
    this->objects.clear(); // TODO degradation stuff
    for (auto &i : cubeTargets) {
        Pose cubePose;
        cubePose.x = 100 + i.x;
        cubePose.y = (13 * NACHI_SUQQQQ) / (0.5 * (i.w + i.h));
        cubePose.probability = i.prob;
        this->objects.push_back(cubePose);
    }
}

void Field::update(LidarScan scan) {
    this->objects.clear(); // TODO degradation stuff
    for (int i = 0; i < 360; ++i) {
        auto dist = scan.getAtAngle(i);
        Pose cubePose;
        cubePose.x = 250 + static_cast<float>(cos((PI * i / 180) - (PI / 2)) * dist / 20);
        cubePose.y = 250 + static_cast<float>(sin((PI * i / 180) - (PI / 2)) * dist / 20);
        cubePose.probability = 0.4;
        this->objects.push_back(cubePose);
    }
}

void Field::tick() {
    render();
}

void Field::render() {
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    int robotRadius = 20;
    int middle_x = img.cols / 2;
    int middle_y = img.rows / 2;
    cv::rectangle(img, cv::Rect(cv::Point2f(me.x - robotRadius / 2, me.y - robotRadius / 2),
                                cv::Size(robotRadius, robotRadius)), cv::Scalar(0, 0, 0), 20);
    cv::line(img, cv::Point(middle_x, middle_y),
             cv::Point2f(middle_x + (cos(me.yaw - (PI / 2)) * robotRadius * 2),
                         middle_y + (sin(me.yaw - (PI / 2)) * robotRadius * 2)),
             cv::Scalar(0, 0, 0),
             3
    );
    for (auto &i : this->objects) {
//        cv::ellipse(img, cv::Point(middle_x, middle_y), cv::Size(img.cols, img.rows), 0, (180/(2*PI))*(atan2(i.y-middle_y, i.x-middle_x))-5, (180/(2*PI))*(atan2(i.y-middle_y, i.x-middle_x))+5, cv::Scalar(50, 255, 255), -1);
        cv::circle(img, cv::Point2f(i.x, i.y), static_cast<int>(i.probability * i.probability * 20),
                   cv::Scalar(20, 190, 190), -1);
    }

    renderedImage = img;
//    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}
