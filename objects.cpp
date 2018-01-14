#include "objects.hpp"
#include <math.h>

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

Pose::Pose() {}

Pose::Pose(const Pose prev, const float measuredAccelForward, const float measuredAccelYaw) {
    x = prev.x + cos(prev.yaw) * prev.rateForward + RAND*0.001-0.0005;
    y = prev.y + cos(prev.yaw) * prev.rateForward + RAND*0.001-0.0005;

    rateForward = prev.rateForward + measuredAccelForward + RAND*0.001-0.0005;

    yaw = prev.yaw + prev.rateYaw;
    rateYaw = prev.rateYaw + measuredAccelYaw;
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
