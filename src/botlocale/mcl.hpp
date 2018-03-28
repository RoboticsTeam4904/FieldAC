#ifndef PROV_MCL_H
#define PROV_MCL_H

#include "../objects.hpp"
#include "lidar.hpp"

#ifndef SAMPLES
#define SAMPLES 1000
#endif

namespace BotLocale {
    Pose* step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelLateral, SensorData prevData, SensorData currData, LidarScan prevScan, LidarScan currScan);
    void tick();
    Pose* init();
    Pose get_best_pose(Pose[SAMPLES], LidarScan scan);
}

#endif
