#include "field.hpp"
#include "./network/network.hpp"
#include <chrono>
#include <thread>
#include <sys/time.h>
#include <math.h>
#include <tuple>
#include "./botlocale/lidar.hpp"
#include <ntcore.h>
#include <networktables/NetworkTable.h>


#define PI 3.14159265
#define TEAM_NUMBER 4904
#define FOCAL_LENGTH 1000
#define NETWORKTABLES_PORT 1735

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
    // For now, just generate a square field as a test
    construct.emplace_back(Segment(0, 0, 0, 10));
    construct.emplace_back(Segment(0, 10, 10, 10));
    construct.emplace_back(Segment(10, 10, 10, 0));
    construct.emplace_back(Segment(10, 0, 0, 0));
    std::printf("%lu\n", construct.size());
    me.x = 250;
    me.y = 250;
    me.yaw = 0; // forward/up
    std::printf("Initializing network tables: https://www.youtube.com/watch?v=dQw4w9WgXcQ...\n");
    nt_inst = nt::GetDefaultInstance();
    nt::StartClientTeam(nt_inst, TEAM_NUMBER, NETWORKTABLES_PORT);
}

void Field::update(std::vector<bbox_t> cubeTargets) {
    this->objects.clear(); // TODO degradation stuff
    // Predict new targets, decrease probability of all, but increase probability of those that are similar to cubeTargets
    for (auto &i : cubeTargets) {
        Pose cubePose;
        cubePose.x = 100 + i.x;
        cubePose.y = 250 - ((13 * FOCAL_LENGTH) / (0.5 * (i.w + i.h)));
        cubePose.yaw = i.x; // TODO pl0x emperical pixels to degrees nikhil
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
        cubePose.yaw = i
        cubePose.probability = 0.4;
        this->objects.push_back(cubePose);
    }
}

void Field::tick() {
    render();
    double x_vals[this->objects.size()];
    double y_vals[this->objects.size()];
    for (int i = 0; i < objects.size(); i++) {
        x_vals[i] = objects[i].x;
        y_vals[i] = objects[i].y;
    }
//    nt->PutNumberArray("vision/x", *x_vals);
//    nt->PutNumberArray("vision/y", *y_vals);
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

void Field::dumpPosesToNT(std::vector<Pose> poses, std::string mainKey) {
    ArrayRef<double> xs;
    ArrayRef<double> ys;
    ArrayRef<double> yaws;
    ArrayRef<double> probs;
    for (const Pose& pose : poses) {
        xs.push_back((250 - pose.x) / 2); //I think this is the conversion from field to feet
        ys.push_back((250 - pose.y) / 2);
        yaws.push_back(pose.yaw);
        probs.push_back(pose.probability);
    }
    mainKey = "/vision/" + mainKey;
    auto x = nt::GetEntry(nt_inst, mainKey + "/xs");
    nt::SetEntryValue(x, nt::Value::MakeDoubleArray(xs));
    auto y = nt::GetEntry(nt_inst, mainKey + "/ys");
    nt::SetEntryValue(y, nt::Value::MakeDoubleArray(xs));
    auto yaw = nt::GetEntry(nt_inst, mainKey + "/yaws");
    nt::SetEntryValue(yaw, nt::Value::MakeDoubleArray(Yaws));
    auto prob = nt::GetEntry(nt_inst, mainKey + "/probs");
    nt::SetEntryValue(prob, nt::Value::MakeDoubleArray(probs));
}
