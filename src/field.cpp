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
#include "vision.hpp"

#define PI 3.14159265
#define TEAM_NUMBER 4904
#define FOCAL_LENGTH 1000   
#define NETWORKTABLES_PORT 1735
#define FIELD_SIZE std::tuple<int, int>(500, 500)
#define DEGRADATION_AMOUNT 0.05
#define FT(CM) (CM * 0.0328084)

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


    std::printf("Field generated.\n\tNumber of segments: %lu\n\tSize: %f x %f\n", construct.size(), field_height, field_width);
    me.x = 250;
    me.y = 250;
    me.yaw = 0; // forward/up
    nt_inst = nt::GetDefaultInstance();
    nt::StartClientTeam(nt_inst, TEAM_NUMBER, NETWORKTABLES_PORT);
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
        auto angles = Vision::pixel_to_angle(i.x, i.y);
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
    this->latest_lidar_scan = scan;
}

//unsure of whether or not it's better practice to but this-> before the functions or not
void Field::tick() {
    this->render();
    this->put_pose_nt(this->objects, "cubes");
    std::printf("published cube data\n");
    map<std::string, double> lData;
    lData.insert(std::make_pair("frontDist", dist_front_obstacle()));
    lData.insert(std::make_pair("x", dist_front_obstacle()));
    lData.insert(std::)
    this->put_values_nt("localization", std::map<std::string, double>{
        "frontDist": dist_front_obstacle(), "x":FT(me.x), "y":FT(me.y)});
    std::printf("published localization data\n");
    this->old_data = latest_data;
    this->get_sensor_data_nt();
    std::printf("got sensor data\n");
    // TODO not sure which accel is forward or lateral
    BotLocale::step(pose_distribution, latest_data.accelX,
                    static_cast<const float>(latest_data.accelY),
                    static_cast<const float>(latest_data.yaw - old_data.yaw),
                    "is this even used?", latest_lidar_scan);
    std::printf("stepped\n");
    me = BotLocale::get_best_pose(pose_distribution);
    std::printf("got best pose (%f, %f)\n",  me.x, me.y);
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
        cv::circle(img, cv::Point2f(p.x, p.y), 1,
                   cv::Scalar(255, 0, 0), -1);
    }

    renderedImage = img;
//    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

float Field::dist_front_obstacle() {
    return this->latest_lidar_scan.getAtAngle(0);
}

// for all nt stuff we might want to use stringrefs insteaf of getEntries
void Field::put_pose_nt(std::vector<Pose> poses, std::string mainKey, std::string parent = "vision") {
    ArrayRef<double> xs, ys, yaws, probs;
    for (const Pose &pose : poses) {
        xs.push_back(FT(this->field_width - pose.x)); 
        ys.push_back(FT(this->field_height - pose.y));
        yaws.push_back(pose.yaw);
        probs.push_back(pose.probability);
    }
    mainKey = "/" + parent + "/" + mainKey;
    nt::SetEntryValue(StringRef(mainKey + "/x"), nt::Value::MakeDoubleArray(xs));
    nt::SetEntryValue(StringRef(mainKey + "/y"), nt::Value::MakeDoubleArray(xs));
    nt::SetEntryValue(StringRef(mainKey + "/yaw"), nt::Value::MakeDoubleArray(yaws));
    nt::SetEntryValue(StringRef(mainKey + "/prob"), nt::Value::MakeDoubleArray(probs));
}

void Field::put_arrays_nt(std::string mainKey, std::map<std::string, ArrayRef<double>> data, std::string parent = "vision") {
    mainKey = "/" + parent + "/" + mainKey + "/";
    for(const auto &i : data) {
        nt::SetEntryValue(StringRef(mainKey + i.first), nt::Value::MakeDoubleArray(i.last));
    }
}

void Field::put_arrays_nt(std::string mainKey, std::string parent, int count, ...) {
    va_list values;
    for (int i = 0; i < count; ++i) {
        std::pair<std::string, ArrayRef<double>> data = va_arg(i, std::pair<std::string, ArrayRef<double>>);
        nt::SetEntryValue(StringRef("/" + parent + "/" + mainKey + "/" + data.first), nt::Value::MakeDoubleArray(data.last));
    }
}

void Field::put_values_nt(std::string mainKey, std::map<std::string, double> data, std::string parent = "vision") {
    mainKey = "/" + parent + "/" + mainKey + "/";
    for(const auto &i : data) {
        nt::SetEntryValue(StringRef(mainKey + i.first), nt::Value::MakeDouble(i.last));
    }
}

void Field::put_values_nt(std::string mainKey, std::string parent, int count, ...) {
    va_list values;
    for (int i = 0; i < count; ++i) {
        std::pair<std::string, double> data = va_arg(i, std::pair<std::string, ArrayRef<double>>);
        nt::SetEntryValue(StringRef("/" + parent + "/" + mainKey + "/" + data.first), nt::Value::MakeDouble(data.last));
    }
}

void Field::put_value_nt(std::string key, double data, std::string parent = "vision") {
    nt::SetEntryValue(StringRef("/" + parent + "/" + key), nt::Value::MakeDouble(data));
}

void Field::put_value_nt(std::string key, ArrayRef<double> data, std::string parent = "vision") {
    nt::SetEntryValue(StringRef("/" + parent + "/" + key), nt::Value::MakeDoubleArray(data));
}

void Field::get_sensor_data_nt() {
    this->latest_data.leftEncoder = nt::GetEntryValue(StringRef("/sensorData/leftEncoder"))->GetDouble();
    this->latest_data.rightEncoder = nt::GetEntryValue(StringRef("/sensorData/rightEncoder"))->GetDouble();
    this->latest_data.accelX = nt::GetEntryValue(StringRef("/sensorData/accelX"))->GetDouble();
    this->latest_data.accelY = nt::GetEntryValue(StringRef("/sensorData/accelY"))->GetDouble();
    this->latest_data.accelZ = nt::GetEntryValue(StringRef("/sensorData/accelZ"))->GetDouble();
    this->latest_data.yaw = nt::GetEntryValue(StringRef("/sensorData/yaw"))->GetDouble();
}

std::map<std::string, double> Field::get_arrays_nt(std::ArrayRef<std::string> keys, std::string parent = "sensorData") {
    std::string mainKey = "/" + parent + "/";
    std::map<std::string, double> data;
    for(const auto &i : data) {
        data[i] = nt::GetEntryValue(StringRef(mainKey + i))->PutNumberArraygetDouble();
    }
    return data;
}