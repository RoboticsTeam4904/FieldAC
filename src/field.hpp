#ifndef PROV_FIELD_H
#define PROV_FIELD_H

#include <vector>
#include <array>
#include "objects.hpp"
#include <tuple>
#include "network/network.hpp"
#include "./botlocale/lidar.hpp"
#include "./botlocale/mcl.hpp"
#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include "botlocale/lidar.hpp"

class Field {
private:
    Field();

    static Field *instance;
    mutable std::mutex scan_mutex;
public:
    static Field *getInstance();
    void run();
    void load();
    void update(std::vector<bbox_t>);
    void update(LidarScan);
    SensorData get_sensor_data();
    void put_vision_data_nt();
    void render();
    float dist_front_obstacle();
    void put_pose_nt(std::vector<Pose>, std::string mainKey, std::string parent);
    void put_arrays_nt(std::string mainKey, std::map<std::string, std::vector<double>> data, std::string parent);
    void put_arrays_nt(std::string mainKey, std::string parent, int count, ...);
    void put_values_nt(std::string mainKey, std::map<std::string, double> data, std::string parent);
    void put_values_nt(std::string mainKey, std::string parent, int count, ...);
    void put_value_nt(std::string key, double data, std::string parent);
    void put_value_nt(std::string key, std::vector<double> data, std::string parent);
    void get_sensor_data_nt();
    std::map<std::string, double> get_values_nt(std::vector<std::string> keys, std::string parent);
    std::vector<Segment> construct;
    std::vector<Pose> objects;
    std::vector<Pose> robots;
    Pose pose_distribution[SAMPLES];
    Pose me;
    cv::Mat renderedImage;
    NT_Inst nt_inst;
    SensorData latest_data;
    SensorData old_data;
    std::deque<LidarScan> lidar_scans;
    double field_width;
    double field_height;
    cv::Mat cameraFrame;
};

#endif