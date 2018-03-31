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
#include <string>
#include <sstream>
// #include <stdarg.h>
#include <cstdarg>
#include <cmath>

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
    nt_inst = nt::GetDefaultInstance();
    nt::StartClientTeam(nt_inst, TEAM_NUMBER, NETWORKTABLES_PORT);
    while (!nt::IsConnected(nt_inst))
        continue;

//    auto randomized = BotLocale::init();
//    for (int i = 0; i < SAMPLES; ++i) {
//        pose_distribution[i] = randomized[i];
//    }

    time_t seconds;
    time(&seconds);
    std::stringstream ss;
    ss << seconds;
    std::string ts = ss.str();
    this->fieldFrameWriter = cv::VideoWriter("field-" + ts + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 10,
                                             cv::Size(field_height, field_width),
                                             true); // rotated so switch field_height and field_width
}

void Field::update(std::vector<bbox_t> cubeTargets) {
//    if (!this->isReady) {
//        return;
//    }
    this->isReady = true;
    this->scan_mutex.lock();
    this->objects.clear(); // TODO degradation stuff
    // Predict new targets, decrease probability of all, but increase probability of those that are similar to cubeTargets
    // Rotate points based on yawRate to get predicted pose
//    int j = 0;
//    for (auto &pose : this->objects) {
//        float s = sin(me.rateYaw);
//        float c = cos(me.rateYaw);
//
//        pose.x -= me.x;
//        pose.y -= me.y;
//
//        float xnew = pose.x * c - pose.y * s;
//        float ynew = pose.x * s + pose.y * c;
//
//        // translate point back:
//        pose.x = xnew + me.x;
//        pose.y = ynew + me.y;
//        pose.probability -= DEGRADATION_AMOUNT;
//        if (pose.probability < 1e-3) {
//            this->objects.erase(this->objects.begin() + j); // delet because it doesn't exist anymore
//        }
//        j++;
//    }
//    for (auto &i : cubeTargets) {
//        Pose cubePose;
//        auto angles = Vision::pixel_to_rad(i.x, i.y, 78, this->cameraFrame); // logitech c920 has 78 degree fov
//        auto distance = i.h; // TODO: some function of the height/width
//        distance = 10; // for now just hard code it to a random value lol
//        cubePose.x = (cos(std::get<0>(angles) + me.yaw) * distance) + me.x;
//        cubePose.y = (sin(std::get<0>(angles) + me.yaw) * distance) + me.y;
//        cubePose.probability = 0.5f + (i.prob / 2);
//        // see if this cube was predicted
//        for (auto &j : this->objects) {
//            if (cubePose == j) { // yep we predicted it (== is overloaded)
//                j = cubePose;
//            } else { // new cube
//                this->objects.push_back(cubePose);
//            }
//        }
//        std::cout << "Number of objects detected: " << objects.size() << std::endl;
//    }
    for (auto &i : cubeTargets) {
        Pose cubePose;
        auto angles = Vision::pixel_to_rad(i.x + (i.w / 2.0F), i.y + (i.h / 2.0F), 78, this->cameraFrame.cols,
                                           this->cameraFrame.rows); // logitech c920 has 78 degree fov
        if (i.w + i.h == 0 || std::get<1>(angles) > 63) {
            continue;
        }
        std::cout << "Found cube at " << std::get<0>(angles) * 180 / M_PI << " degrees" << std::endl;
        float distance = (CUBE_SIZE * FOCAL_LENGTH) / (0.5 * (i.h + i.w));
        // cubePose.x = (cos(std::get<0>(angles) + me.yaw) * distance) + me.x;
        // cubePose.y = (sin(std::get<0>(angles) + me.yaw) * distance) + me.y;
        cubePose.dist = distance;
        cubePose.relangle = std::get<0>(angles);
        cubePose.probability = 0.5f + (i.prob / 2);
        this->objects.push_back(cubePose);
        std::cout << "Number of objects detected: " << objects.size() << std::endl;
    }
    this->scan_mutex.unlock();
}

void Field::update(LidarScan scan) {
    auto sensorData = this->get_sensor_data();
    this->scan_mutex.lock();
    scan.yaw = (float) sensorData.yaw;
//    this->lidar_scans.push_back(scan);
//    if (this->lidar_scans.size() > 5) {
//        this->lidar_scans.pop_front();
//    }
    this->scan_mutex.unlock();
}

void Field::put_vision_data_nt() {
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

SensorData Field::get_sensor_data() {
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
    auto yaw = nt::GetEntry(nt_inst, "/sensorData/yaw");
    this->latest_data.yaw = (nt::GetEntryValue(yaw)->GetDouble()) * PI / 180;
//    this->latest_data.leftEncoder = 0;
//    this->latest_data.rightEncoder = 0;
//    this->latest_data.accelX = 0;
//    this->latest_data.accelY = 0;
//    this->latest_data.accelZ = 0;
//
    this->latest_data.accelX = latest_data.accelX * IMU_TO_CM_S2;
    this->latest_data.accelY = latest_data.accelY * IMU_TO_CM_S2;
    this->latest_data.accelZ = latest_data.accelZ * IMU_TO_CM_S2;
    return this->latest_data;
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
        cv::circle(img, cv::Point2f(i.x, i.y), static_cast<int>(i.probability * i.probability * 20),
                   cv::Scalar(20, 190, 190), -1);
    }

    for (auto p : this->pose_distribution) {
        cv::circle(img, cv::Point2f(p.x, p.y), 3,
                   cv::Scalar(p.probability, p.probability, p.probability), -1);
        cv::line(img, cv::Point2f(p.x, p.y), cv::Point2f(p.x + (p.dx), p.y + (p.dy)),
                 cv::Scalar(128, 128, 0), 1);
    }
//
//    for (auto m : this->lidar_scans.front().measurements) {
//        double x_pos = cos(std::get<0>(m) * PI / 180 + me.yaw - (PI / 2)) * std::get<1>(m) + me.x;
//        double y_pos = sin(std::get<0>(m) * PI / 180 + me.yaw - (PI / 2)) * std::get<1>(m) + me.y;
//        cv::circle(img, cv::Point2d(x_pos, y_pos), 2,
//                   cv::Scalar(0, 255, 0), -1);
//    }
//    for (auto m : this->lidar_scans.at(lidar_scans.size() - 2).measurements) {
//        double x_pos = cos(std::get<0>(m) * PI / 180 + me.yaw - (PI / 2)) * std::get<1>(m) + me.x;
//        double y_pos = sin(std::get<0>(m) * PI / 180 + me.yaw - (PI / 2)) * std::get<1>(m) + me.y;
//        cv::circle(img, cv::Point2d(x_pos, y_pos), 2,
//                   cv::Scalar(50, 128, 50), -1);
//    }
//    this->lidar_scans.back().raytrace_visual(me, img);
    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);
//    cv::resize(img, img, cv::Size(0.5, 0.5));

    renderedImage = img;
    this->fieldFrameWriter.write(renderedImage);
//    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

float Field::dist_front_obstacle() {
    scan_mutex.lock();
//    auto val = FT(this->lidar_scans.front().getAtAngle(0));
    scan_mutex.unlock();
    auto val = 0;
    return val;
}

std::vector<double> Field::dist_major_angles() {
    std::vector<double> major_angles;
    scan_mutex.lock();
    for(int i = 0; i < 8; i++) {
        major_angles[i] = FT(this->lidar_scans.front().getAtAngle(i * 45));
    }
    scan_mutex.unlock();
};

void Field::put_pose_nt(std::vector<Pose> poses, std::string mainKey, std::string parent = "vision") {
    std::vector<double> xs, ys, dists, relangles;
    if (poses.size() < 1) {
        return;
    }
    for (const Pose pose : poses) {
        xs.push_back(FT(this->field_width - pose.x));
        ys.push_back(FT(this->field_height - pose.y));
        dists.push_back(FT(pose.dist));
        relangles.push_back(pose.relangle * 180 / M_PI);
    }
    //hold on bois, its about to get baaaaad
    double s = 9999999;
    int e;
    for (int i = 0; i < relangles.size(); ++i) {
        if (objects[i].dist < s) {
            s = objects[i].dist;
            e = i;
        }
    }
    if (xs.size() > 0) {
        s = xs[0];
        xs[0] = xs[e];
        xs[e] = s;
        s = ys[0];
        ys[0] = ys[e];
        ys[e] = s;
        s = dists[0];
        dists[0] = dists[e];
        dists[e] = s;
        s = relangles[0];
        relangles[0] = relangles[e];
        relangles[e] = s;
    }
    mainKey = "/" + parent + "/" + mainKey;
    nt::SetEntryValue(mainKey + "/x", nt::Value::MakeDoubleArray(xs));
    nt::SetEntryValue(mainKey + "/y", nt::Value::MakeDoubleArray(ys));
    nt::SetEntryValue(mainKey + "/distance", nt::Value::MakeDoubleArray(dists));
    nt::SetEntryValue(mainKey + "/relangle", nt::Value::MakeDoubleArray(relangles));

}

void Field::put_arrays_nt(std::string mainKey, std::map<std::string, std::vector<double>> data,
                          std::string parent = "vision") {
    mainKey = "/" + parent + "/" + mainKey + "/";
    for (const auto &i : data) {
        nt::SetEntryValue(mainKey + i.first, nt::Value::MakeDoubleArray(i.second));
    }
}

// void Field::put_arrays_nt(std::string mainKey, std::string parent, int count, ...) {
//     count *= 2;
//     va_list values;
//     va_start(values, count);
//     for (int i = 0; i < count; i += 2) {
//         std::string key = va_arg(values, std::string);
//         std::vector<double> data = va_arg(values, std::vector<double>);
//         nt::SetEntryValue("/" + parent + "/" + mainKey + "/" + key, nt::Value::MakeDoubleArray(data));
//     }
//     va_end(values);
// }

void Field::put_values_nt(std::string mainKey, std::map<std::string, double> data, std::string parent = "vision") {
    mainKey = "/" + parent + "/" + mainKey + "/";
    for (const auto &i : data) {
        nt::SetEntryValue(mainKey + i.first, nt::Value::MakeDouble(i.second));
    }
}

void Field::put_values_nt(std::string mainKey, std::string parent, int count, ...) {
    count *= 2;
    va_list values;
    va_start(values, count);
    for (int i = 0; i < count; i += 2) {
        char *key = va_arg(values, char*);
        double data = va_arg(values, double);
        nt::SetEntryValue("/" + parent + "/" + mainKey + "/" + key, nt::Value::MakeDouble(data));
    }
    va_end(values);
}

void Field::put_value_nt(std::string key, double data, std::string parent = "vision") {
    nt::SetEntryValue("/" + parent + "/" + key, nt::Value::MakeDouble(data));
}

void Field::put_value_nt(std::string key, std::vector<double> data, std::string parent = "vision") {
    nt::SetEntryValue("/" + parent + "/" + key, nt::Value::MakeDoubleArray(data));
}

void Field::get_sensor_data_nt() {
    this->latest_data.leftEncoder = nt::GetEntryValue("/sensorData/leftEncoder")->GetDouble();
    this->latest_data.rightEncoder = nt::GetEntryValue("/sensorData/rightEncoder")->GetDouble();
    this->latest_data.accelX = nt::GetEntryValue("/sensorData/accelX")->GetDouble();
    this->latest_data.accelY = nt::GetEntryValue("/sensorData/accelY")->GetDouble();
    this->latest_data.accelZ = nt::GetEntryValue("/sensorData/accelZ")->GetDouble();
    this->latest_data.yaw = nt::GetEntryValue("/sensorData/yaw")->GetDouble();
}

std::map<std::string, double> Field::get_values_nt(std::vector<std::string> keys, std::string parent = "sensorData") {
    std::string mainKey = "/" + parent + "/";
    std::map<std::string, double> data;
    for (const auto &i : data) {
        data[i.first] = nt::GetEntryValue(mainKey + i.first)->GetDouble();
    }
    return data;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

void Field::run() {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    while (true) {
        while (this->objects.empty() || !this->isReady) {
            continue;
        }
        this->scan_mutex.lock();
        this->put_pose_nt(this->objects, "cubes", "pose");
        std::printf("published cube data\n");
        this->put_values_nt("localization", "vision", 3, "frontObsticalDist", dist_front_obstacle(), "x", me.x, "y",
                            me.y);
        
//        this->put_arrays_nt("localization", {{"major_angles", this->dist_major_angles()}}, "vision");
        std::printf("published localization data\n");
        this->scan_mutex.unlock();
        this->old_data = latest_data;
        this->get_sensor_data();
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

#pragma clang diagnostic pop