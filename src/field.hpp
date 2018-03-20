#ifndef PROV_FIELD_H
#define PROV_FIELD_H

#include <vector>
#include <array>
#include "objects.hpp"
#include <tuple>
#include "network/network.hpp"
#include "./botlocale/lidar.hpp"

struct Segment {
    Segment(int xi, int yi, int xf, int yf);
    Segment(std::tuple<int, int> start, std::tuple<int, int> end);
    std::tuple<int, int> start;
    std::tuple<int, int> end;
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
    void render();
    std::vector<Segment> construct;
    std::vector<Pose> objects;
    std::vector<Pose> robots;
    Pose me;
    cv::Mat renderedImage;
};

#endif