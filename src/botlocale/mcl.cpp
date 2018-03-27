#include "mcl.hpp"
#include <tuple>
#include <cmath>
#include <vector>
#include <iostream>
#include "lidar.hpp"
#include "../field.hpp"

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

Pose *BotLocale::init() {
    static Pose poses[SAMPLES];
    for (auto &pose : poses) {
        pose.seed();
    }
    return poses;
}

Pose* BotLocale::step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelLateral,
                     const float measuredAccelYaw, SensorData sensorData, LidarScan scan) {
    Pose n[SAMPLES];
    float weights[SAMPLES];
    float weightsSum = 0;
    double ms;
    std::clock_t a = std::clock();
    for (int i = 0; i < SAMPLES; i++) {
        n[i] = Pose(input[i], measuredAccelForward, measuredAccelLateral, measuredAccelYaw);
        weights[i] = static_cast<float>(1 / (scan.raytrace(n[i]) + (n[i].yaw - sensorData.yaw))); //TODO who knows
        n[i].probability = weights[i];
        weightsSum += weights[i];
    }
    ms = (std::clock() - a) / (double) (CLOCKS_PER_SEC * 2.7 / 1000);
    std::cout << "Total raytrace time: " << ms << "ms" << std::endl;
    std::clock_t start = std::clock();
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
    ms = (std::clock() - start) / (double) (CLOCKS_PER_SEC * 2.7 / 1000);
    std::cout << "Average MCL time: " << ms/SAMPLES << "ms" << std::endl;
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