#ifndef PROV_FIELD_H
#define PROV_FIELD_H

// STD
#include <vector>
#include <array>
#include <tuple>
#include <map>
// Darknet
#include <yolo_v2_class.hpp>
#include <opencv2/opencv.hpp>
// FieldAC
#include "objects.hpp"

class Field {
public:
    std::vector<Segment> construct;
    std::vector<Pose> objects;
    std::vector<Pose> robots;
    Pose self;
    SensorData data_curr;
    SensorData data_prev;
    double field_width;
    double field_height;
private:
    Field();
    static Field *instance;
    mutable std::mutex objects_mutex;
    bool isReady;
public:
    static Field *getInstance();
    void run();
    void load();
    void update(std::vector<Pose>);
    void render();
    std::vector<Pose> finalizeObjects();
    cv::Mat renderMat;
    cv::VideoWriter renderWriter;

    inline static float CM_TO_FT(float CM) {
        return CM * 0.0328084F;
    };
};

#endif