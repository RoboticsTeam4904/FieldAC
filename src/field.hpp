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


struct SensorData {
    double leftEncoder;
    double rightEncoder;
    double accelX;
    double accelY;
    double accelZ;
    double yaw;
};

class Field {
private:
    Field();
    static Field* instance;
public:
    static Field* getInstance();
    void load();
    void update(std::vector<bbox_t>);
    void update(LidarScan);
    void tick();
    void get_sensor_data();
    void put_vision_data();
    void render();
    std::vector<Segment> construct;
    std::vector<Pose> objects;
    std::vector<Pose> robots;
    Pose me;
    cv::Mat renderedImage;
    NT_Inst nt_inst;
    SensorData latest_data;
};

#endif