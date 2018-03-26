#include "mcl.hpp"
#include <tuple>
#include <cmath>
#include <vector>
#include "lidar.hpp"

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

Pose *BotLocale::init() {
    static Pose poses[SAMPLES];
    for (auto &pose : poses) {
        pose.seed();
    }
    return poses;
}

Pose* BotLocale::step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelLateral,
                     const float measuredAccelYaw, std::string sensorData, LidarScan scan) {
    Pose n[SAMPLES];
    float weights[SAMPLES];
    float weightsSum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        n[i] = Pose(input[i], measuredAccelForward, measuredAccelLateral, measuredAccelYaw);
        weights[i] = 1.0f/static_cast<float>(scan.raytrace(n[i])); //TODO who knows
        n[i].probability = weights[i];
        weightsSum += weights[i];
    }
    for (int i = 0; i < SAMPLES; i++) {
        float weight = weightsSum * RAND;
        for (int j = 0; j < SAMPLES; j++) {
            weight -= weights[j];
            if (weight < 0) {
                input[i] = n[j];
                break;
            }
        }
    }
    return input;
}

Pose BotLocale::get_best_pose(Pose input[SAMPLES]) {
    Pose total_pose;
    for (int i = 0; i < SAMPLES; ++i) {
        total_pose = input[i] + total_pose;
    }
    Pose average_pose = total_pose / SAMPLES;
    return average_pose;
}