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
#include "./botlocale/mcl.hpp"

#define PI 3.14159265
#define NETWORKTABLES_PORT 1735
#define TEAM_NUMBER 4904
#define NACHI_SUQQQQ 1000


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
    construct.emplace_back(Segment(0, 0, 0, 100));
    construct.emplace_back(Segment(0, 100, 100, 100));
    construct.emplace_back(Segment(100, 100, 100, 0));
    construct.emplace_back(Segment(100, 0, 0, 0));
    std::printf("%lu\n", construct.size());
    me.x = 250;
    me.y = 250;
    me.yaw = 0; // forward/up
    nt_inst = nt::GetDefaultInstance();
    nt::StartClientTeam(nt_inst, TEAM_NUMBER, NETWORKTABLES_PORT);

    auto randomized = BotLocale::init();
    for (int i = 0; i < SAMPLES; ++i) {
        pose_distribution[i] = randomized[i];
    }
}

void Field::update(std::vector<bbox_t> cubeTargets) {
    this->objects.clear(); // TODO degradation stuff
    // Predict new targets, decrease probability of all, but increase probability of those that are similar to cubeTargets
    for (auto &i : cubeTargets) {
        Pose cubePose;
        cubePose.x = 100 + i.x; // fix. this should be cos(pixel-to-angle) * width
        cubePose.y = 250 - ((13 * NACHI_SUQQQQ) / (0.5 * (i.w + i.h))); // this should be size of cube * sin(pixel-to-angle)
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
    this->put_vision_data();
    this->old_data = latest_data;
    this->get_sensor_data();
    // TODO not sure which accel is forward or lateral
    BotLocale::step(pose_distribution, latest_data.accelX, latest_data.accelY, latest_data.yaw-old_data.yaw, "is this even used?");
}
void Field::put_vision_data() {
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    for (int i = 0; i < objects.size(); i++) {
        x_vals.push_back(objects[i].x);
        y_vals.push_back(objects[i].y);
    }
    auto x = nt::GetEntry(nt_inst, "/vision/x");
    nt::SetEntryValue(x, nt::Value::MakeDoubleArray(x_vals));
    auto y = nt::GetEntry(nt_inst, "/vision/y");
    nt::SetEntryValue(y, nt::Value::MakeDoubleArray(y_vals));
}
void Field::get_sensor_data() {
    auto leftEncoder_table = nt::GetEntry(nt_inst, "/sensorData/leftEncoder");
    this->latest_data.leftEncoder = nt::GetEntryValue(leftEncoder_table)->GetDouble();
    auto rightEncoder_table = nt::GetEntry(nt_inst, "/sensorData/rightEncoder");
    this->latest_data.rightEncoder = nt::GetEntryValue(rightEncoder_table)->GetDouble();
    auto accelX_table = nt::GetEntry(nt_inst, "/sensorData/accelX");
    this->latest_data.accelX = nt::GetEntryValue(accelX_table)->GetDouble();
    auto accelY_table = nt::GetEntry(nt_inst, "/sensorData/accelY");
    this->latest_data.accelY = nt::GetEntryValue(accelY_table)->GetDouble();
    auto accelZ_table = nt::GetEntry(nt_inst, "/sensorData/accelZ");
    this->latest_data.accelZ = nt::GetEntryValue(accelZ_table)->GetDouble();
}

void Field::render() {
    cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    int robotRadius = 20;
    int middle_x = img.cols / 2;
    int middle_y = img.rows / 2;
    cv::rectangle(img, cv::Rect(cv::Point2f(me.x - robotRadius / 2, me.y - robotRadius / 2),
                                cv::Size(robotRadius, robotRadius)), cv::Scalar(0, 0, 0), 20);
    cv::line(img, cv::Point(middle_x, middle_y),
             cv::Point2f(static_cast<float>(middle_x + (cos(me.yaw - (PI / 2)) * robotRadius * 2)),
                         static_cast<float>(middle_y + (sin(me.yaw - (PI / 2)) * robotRadius * 2))),
             cv::Scalar(0, 0, 0),
             3
    );
    for (auto &line : this->construct) {
        // draw lines to represent field
        // transform segment around robot
        auto transformed = line.rotate(me.x, me.y, me.yaw);
        cv::line(img, tuple_to_point(transformed.start), tuple_to_point(transformed.end), cv::Scalar(0, 0, 0), 3);
    }
    for (auto &i : this->objects) {
//        cv::ellipse(img, cv::Point(middle_x, middle_y), cv::Size(img.cols, img.rows), 0, (180/(2*PI))*(atan2(i.y-middle_y, i.x-middle_x))-5, (180/(2*PI))*(atan2(i.y-middle_y, i.x-middle_x))+5, cv::Scalar(50, 255, 255), -1);
        cv::circle(img, cv::Point2f(i.x, i.y), static_cast<int>(i.probability * i.probability * 20),
                   cv::Scalar(20, 190, 190), -1);
    }

    renderedImage = img;
//    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}
