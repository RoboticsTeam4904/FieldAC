#ifndef PROV_FIELD_H
#define PROV_FIELD_H

#include <vector>
#include <array>
#include "objects.hpp"

struct Segment {
    Segment(int xi, int yi, int xf, int yf);
    Segment(std::array<int, 2> start, std::array<int, 2> end);
    std::array<int, 2> start;
    std::array<int, 2> end;
};

class Field {
private:
    std::vector<Segment> construct;
public:
    Field();
    static Field* load();
    std::vector<Pose> objects;
    std::vector<Pose> robots;
};

#endif