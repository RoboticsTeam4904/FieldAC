#include "objects.hpp"
#include <math.h>

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

#define ZRAND (RAND - 0.5)

#define VELOCITY_NOISE 0.0001

#define YAW_RATE_NOISE 0.0001

Pose::Pose() = default;

Pose::Pose(const Pose prev, const float measuredAccelForward, const float measuredAccelLateral, const float measuredAccelYaw) {
    x = static_cast<float>(prev.x + prev.dx);
    y = static_cast<float>(prev.y + prev.dy);

    dx=static_cast<float>(prev.dx + measuredAccelForward * cos(prev.yaw) - measuredAccelLateral * sin(prev.yaw) + ZRAND * VELOCITY_NOISE);
    dy=static_cast<float>(prev.dy + measuredAccelForward * sin(prev.yaw) + measuredAccelLateral * cos(prev.yaw) + ZRAND * VELOCITY_NOISE);

    yaw = prev.yaw + prev.rateYaw;
    rateYaw = prev.rateYaw + measuredAccelYaw + ZRAND * YAW_RATE_NOISE;
}

void Pose::seed() {
    x=RAND;
    y=RAND;
    rateForward=RAND;
    rateYaw = 0;
    yaw = 0;
}

float Pose::plausibility(const std::string sensorData) {
    if(x<-10 || x>10 || y<-10 || y>10) {
        return 0.1;
    }
    return 1;
}
