#include "field.hpp"
#include "./network/network.hpp"
#include <chrono>
#include <thread>
#include <sys/time.h>
#include <tuple>
#include "./botlocale/lidar.hpp"
#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include "./botlocale/mcl.hpp"
#include "vision.hpp"
#include <cmath>

#define PI 3.14159265
#define NETWORKTABLES_PORT 1735
#define TEAM_NUMBER 4904
#define NACHI_SUQQQQ 1000
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
    nt_inst = nt::GetDefaultInstance();
    nt::StartClient(nt_inst, "localhost", NETWORKTABLES_PORT);
    while (!nt::IsConnected(nt_inst))
        continue;

    auto randomized = BotLocale::init();
    for (int i = 0; i < SAMPLES; ++i) {
        pose_distribution[i] = randomized[i];
    }
}

void Field::update(std::vector<bbox_t> cubeTargets) {
    this->objects.clear(); // TODO degradation stuff
    // Predict new targets, decrease probability of all, but increase probability of those that are similar to cubeTargets
    // Rotate points based on yawRate to get predicted pose
    int i = 0;
    for (auto &pose : this->objects) {
        float s = sin(me.rateYaw);
        float c = cos(me.rateYaw);

        pose.x -= me.x;
        pose.y -= me.y;

        float xnew = pose.x * c - pose.y * s;
        float ynew = pose.x * s + pose.y * c;

        // translate point back:
        pose.x = xnew + me.x;
        pose.y = ynew + me.y;
        pose.probability -= DEGRADATION_AMOUNT;
        if (pose.probability < 1e-3) {
            this->objects.erase(this->objects.begin() + i); // delet because it doesn't exist anymore
        }
        i++;
    }
    for (auto &i : cubeTargets) {
        Pose cubePose;
        auto angles = Vision::pixel_to_angle(i.x, i.y, 78, this->cameraFrame); // logitech c920 has 78 degree fov
        auto distance = i.h; // TODO: some function of the height/width
        distance = 10; // for now just hard code it to a random value lol
        cubePose.x = (cos(std::get<0>(angles) + me.yaw) * distance) + me.x;
        cubePose.y = (sin(std::get<0>(angles) + me.yaw) * distance) + me.y;
        cubePose.probability = 0.5f + (i.prob / 2);
        // see if this cube was predicted
        for (auto &j : this->objects) {
            if (cubePose == j) { // yep we predicted it (== is overloaded)
                j = cubePose;
            } else { // new cube
                this->objects.push_back(cubePose);
            }
        }
    }
}

void Field::update(LidarScan scan) {
    auto old = latest_lidar_scan;
    this->latest_lidar_scan = scan;
    this->scan_mutex.lock();
    this->old_lidar_scan = old;
    this->scan_mutex.unlock();
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
void Field::run() {
    this->old_lidar_scan = this->latest_lidar_scan;
    while (true) {
        this->put_vision_data();
        std::printf("published vision data\n");
        this->old_data = latest_data;
        this->get_sensor_data();
        std::printf("got sensor data\n");
        // TODO not sure which accel is forward or lateral
        std::clock_t start = std::clock();
        bool lidarIsReady = false;
        for (auto a : latest_lidar_scan.measurements) {
            if (std::get<0>(a) != 0) {
                lidarIsReady = true;
            }
        }
        if (!lidarIsReady) {
            continue;
        }
        for (auto &p : pose_distribution) {
            p.yaw = static_cast<float>(0);
        }
        this->scan_mutex.lock();
        BotLocale::step(pose_distribution, static_cast<const float>(0),
                        static_cast<const float>(0),
                        old_data, latest_data,
                        old_lidar_scan, latest_lidar_scan);
        render();
        this->scan_mutex.unlock();
        int ms = (std::clock() - start) / (double) (CLOCKS_PER_SEC * 2.7 / 1000);
        int fps = 1000 / ms;
        std::cout << "Stepped in " << ms << "ms (" << fps << " hz)" << std::endl;
        me = BotLocale::get_best_pose(pose_distribution);
        me.yaw = static_cast<float>((latest_data.yaw + me.yaw) / 2);
        std::printf("got best pose (%f, %f) at %f degrees moving (%f, %f) and turning %f\n", me.x, me.y,
                    me.yaw * 180 / PI, me.dx, me.dy, me.rateYaw * 180 / PI);
    }
}
#pragma clang diagnostic pop

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
//    auto leftEncoder_table = nt::GetEntry(nt_inst, "/sensorData/leftEncoder");
//    this->latest_data.leftEncoder = nt::GetEntryValue(leftEncoder_table)->GetDouble();
//    auto rightEncoder_table = nt::GetEntry(nt_inst, "/sensorData/rightEncoder");
//    this->latest_data.rightEncoder = nt::GetEntryValue(rightEncoder_table)->GetDouble();
//    auto accelX_table = nt::GetEntry(nt_inst, "/sensorData/accelX");
//    this->latest_data.accelX = nt::GetEntryValue(accelX_table)->GetDouble();
//    auto accelY_table = nt::GetEntry(nt_inst, "/sensorData/accelY");
//    this->latest_data.accelY = nt::GetEntryValue(accelY_table)->GetDouble();
//    auto accelZ_table = nt::GetEntry(nt_inst, "/sensorData/accelZ");
//    this->latest_data.accelZ = nt::GetEntryValue(accelZ_table)->GetDouble();
//    auto yaw = nt::GetEntry(nt_inst, "/sensorData/yaw");
//    this->latest_data.yaw = (nt::GetEntryValue(yaw)->GetDouble()) * PI / 180;
    this->latest_data.leftEncoder = 0;
    this->latest_data.rightEncoder = 0;
    this->latest_data.accelX = 0;
    this->latest_data.accelY = 0;
    this->latest_data.accelZ = 0;

    this->latest_data.accelX = latest_data.accelX * IMU_TO_CM_S2;
    this->latest_data.accelY = latest_data.accelY * IMU_TO_CM_S2;
    this->latest_data.accelZ =latest_data.accelZ * IMU_TO_CM_S2;
}

void Field::render() {
    cv::Mat img(field_height, field_width, CV_8UC3, cv::Scalar(255, 255, 255));
    int robotRadius = 20;
    int middle_x = img.cols / 2;
    int middle_y = img.rows / 2;

    cv::rectangle(img, cv::Rect(cv::Point2f(me.x - robotRadius / 2, me.y - robotRadius / 2),
                                cv::Size(robotRadius, robotRadius)), cv::Scalar(0, 0, 0), 20);
    cv::line(img, cv::Point(me.x, me.y),
             cv::Point2f(static_cast<float>(me.x + (cos(me.yaw - (PI / 2)) * robotRadius * 2)),
                         static_cast<float>(me.y + (sin(me.yaw - (PI / 2)) * robotRadius * 2))),
             cv::Scalar(0, 0, 0),
             3
    );
    for (auto &line : this->construct) {
//        // draw lines to represent field
//        // transform segment around robot
//        auto transformed = line.rotate(me.x, me.y, me.yaw);
//        cv::line(img, tuple_to_point(transformed.start), tuple_to_point(transformed.end), cv::Scalar(0, 0, 0), 3);
        cv::line(img, tuple_to_point(line.start), tuple_to_point(line.end), cv::Scalar(0, 0, 0), 3);
    }
    for (auto &i : this->objects) {
//        cv::ellipse(img, cv::Point(middle_x, middle_y), cv::Size(img.cols, img.rows), 0, (180/(2*PI))*(atan2(i.y-middle_y, i.x-middle_x))-5, (180/(2*PI))*(atan2(i.y-middle_y, i.x-middle_x))+5, cv::Scalar(50, 255, 255), -1);
        cv::circle(img, cv::Point2f(i.x, i.y), static_cast<int>(i.probability * i.probability * 20),
                   cv::Scalar(20, 190, 190), -1);
    }

    for (auto p : this->pose_distribution) {
        cv::circle(img, cv::Point2f(p.x, p.y), 3,
                   cv::Scalar(p.probability, p.probability, p.probability), -1);
        cv::line(img, cv::Point2f(p.x, p.y), cv::Point2f(p.x+(p.dx), p.y+(p.dy)),
                 cv::Scalar(128, 128, 0), 1);
    }

    for (auto m : this->latest_lidar_scan.measurements) {
        double x_pos = cos(std::get<0>(m) * PI / 180 + me.yaw - (PI / 2)) * std::get<1>(m) + me.x;
        double y_pos = sin(std::get<0>(m) * PI / 180 + me.yaw - (PI / 2)) * std::get<1>(m) + me.y;
        cv::circle(img, cv::Point2d(x_pos, y_pos), 2,
                   cv::Scalar(0, 255, 0), -1);
    }
    for (auto m : this->old_lidar_scan.measurements) {
        double x_pos = cos(std::get<0>(m) * PI / 180 + me.yaw - (PI / 2)) * std::get<1>(m) + me.x;
        double y_pos = sin(std::get<0>(m) * PI / 180 + me.yaw - (PI / 2)) * std::get<1>(m) + me.y;
        cv::circle(img, cv::Point2d(x_pos, y_pos), 2,
                   cv::Scalar(50, 128, 50), -1);
    }
    this->latest_lidar_scan.raytrace_visual(me, img);

    renderedImage = img;
//    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}
