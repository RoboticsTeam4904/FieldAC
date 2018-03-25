#ifndef PROV_MCL_H
#define PROV_MCL_H

#include "../objects.hpp"

#ifndef SAMPLES
#define SAMPLES 1000
#endif

namespace BotLocale {
    void step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelLateral,const float measuredAccelYaw, std::string sensorData);
    void tick();
    Pose* init();
    Pose get_best_pose(Pose[SAMPLES]);
}

struct Segment {
    Segment(double xi, double yi, double xf, double yf);
    Segment(std::tuple<double, double> start, std::tuple<double, double> end);
    Segment rotate(double, double, float);
    std::tuple<double, double> start;
    std::tuple<double, double> end;
};

#endif
