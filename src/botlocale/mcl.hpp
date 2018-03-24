#ifndef PROV_MCL_H
#define PROV_MCL_H

#include "../objects.hpp"
#include "lidar.hpp"

#ifndef SAMPLES
#define SAMPLES 1000
#endif

namespace BotLocale {
    void step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelLateral,const float measuredAccelYaw, std::string sensorData);
    void tick();
}

struct Segment {
    Segment(int xi, int yi, int xf, int yf);
    Segment(std::tuple<int, int> start, std::tuple<int, int> end);
    Segment rotate(int, int, float);
    std::tuple<int, int> start;
    std::tuple<int, int> end;
};

#endif
