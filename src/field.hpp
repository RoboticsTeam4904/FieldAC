#ifndef PROV_FIELD_H
#define PROV_FIELD_H

// STD
#include <vector>
#include <array>
#include <tuple>
#include <map>
// WPISUITE
#include <ntcore.h>
#include <networktables/NetworkTable.h>
// Darknet
#include <yolo_v2_class.hpp>
#include <opencv2/opencv.hpp>
// FieldAC
#include "objects.hpp"
#include "botlocale/lidar.hpp"

class Field {
private:
    Field();

    static Field *instance;
    mutable std::mutex scan_mutex;
    bool isReady;
public:
    static Field *getInstance();
    void run();
    void load();
    void update(std::vector<bbox_t>);
    std::vector<Segment> construct;
    std::vector<Pose> objects;
    std::vector<Pose> robots;
    Pose me;
    SensorData latest_data;
    SensorData old_data;
    double field_width;
    double field_height;
};

#endif