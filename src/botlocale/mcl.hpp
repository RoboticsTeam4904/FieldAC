#ifndef PROV_MCL_H
#define PROV_MCL_H

#include "../objects.hpp"
#include "lidar.hpp"

#ifndef SAMPLES
#define SAMPLES 1000
#endif

namespace BotLocale {
    void step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelLateral,const float measuredAccelYaw, std::string sensorData, LidarScan scan);
    void tick();
    Pose* init();
    Pose get_best_pose(Pose[SAMPLES]);
}

#endif
