#ifndef OBJECTS_H
#define OBJECTS_H

#include <string>

struct Pose {
    Pose();
    Pose(const Pose prev, const float measuredAccelForward, const float measuredAccelLateral, const float measuredAccelYaw);
    float x;
    float y;
    float dx;
    float dy;
    float yaw;
    float rateYaw;
    float plausibility(const std::string sensorData);
    void seed();
};

#endif
