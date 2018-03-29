#ifndef PROV_MCL_H
#define PROV_MCL_H

#include "../objects.hpp"
#include "lidar.hpp"

#ifndef SAMPLES
#define SAMPLES 1000
#endif

namespace BotLocale {
    Pose *step(Pose input[1000], SensorData prevData, SensorData currData, std::deque<LidarScan> scans);
    void tick();
    Pose* init();
    Pose get_best_pose(Pose[SAMPLES], LidarScan scan);
}

#endif
