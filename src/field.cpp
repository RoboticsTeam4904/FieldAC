// FieldAC
#include "vision.hpp"
#include "field.hpp"
#include "botlocale/lidar.hpp"
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

#define FT(CM) (CM * 0.0328084)
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

void Field::update(std::vector<Pose> targetObjects) {
    this->objects_mutex.lock();
    this->objects.clear();
    std::move(targetObjects.begin(), targetObjects.end(), this->objects.begin());
    this->objects_mutex.unlock();
}

std::vector<Pose> Field::finalizeObjects() {
    std::vector<Pose> cleanObjects;
    this->objects_mutex.lock();
    for(const Pose pose : this->objects) {
        Pose cleanPose;
        cleanPose.x = Field::CM_TO_FT(this->field_width - pose.x);
        cleanPose.y = Field::CM_TO_FT(this->field_height - pose.y);
        cleanPose.dist = Field::CM_TO_FT(pose.dist);
        cleanPose.relangle = pose.relangle * 180 / M_PI;
        cleanObjects.push_back(cleanPose);
    }
    this->objects_mutex.unlock();
    bool (*sortPred)(Pose, Pose) = [](Pose a, Pose b) { return a.dist > b.dist; };
    std::sort(cleanObjects.begin(), cleanObjects.end(), sortPred);
    return cleanObjects;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

void Field::run() {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    while (true) {
        while (this->objects.empty() || !this->isReady) {
            continue;
        }
        this->data_prev = data_curr;
//        this->get_sensor_data();
//        std::printf("got sensor data\n");
        // TODO not sure which accel is forward or lateral
        std::clock_t start = std::clock();
        bool lidarIsReady = true;
    }
}

void Field::render() {
    cv::Mat img(field_height, field_width, CV_8UC3, cv::Scalar(255, 255, 255));
    int robotRadius = 20;
    int middle_x = img.cols / 2;
    int middle_y = img.rows / 2;

    cv::rectangle(img, cv::Rect(cv::Point2f(self.x - robotRadius / 2, self.y - robotRadius / 2),
                                cv::Size(robotRadius, robotRadius)), cv::Scalar(0, 0, 0), 20);
    cv::line(img, cv::Point(self.x, self.y),
             cv::Point2f(static_cast<float>(self.x + (cos(self.yaw - (M_PI / 2)) * robotRadius * 2)),
                         static_cast<float>(self.y + (sin(self.yaw - (M_PI / 2)) * robotRadius * 2))),
             cv::Scalar(0, 0, 0),
             3
    );
    for (auto &line : this->construct) {
        cv::line(img, tuple_to_point(line.start), tuple_to_point(line.end), cv::Scalar(0, 0, 0), 3);
    }
    for (auto &i : this->objects) {
        cv::circle(img, cv::Point2f(i.x, i.y), static_cast<int>(i.probability * i.probability * 20),
                   cv::Scalar(20, 190, 190), -1);
    }

    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);

    this->renderMat = img;
    this->renderWriter.write(renderMat);
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
    self.x = 0;
    self.y = 0;
    self.yaw = 0;
}

#pragma clang diagnostic pop