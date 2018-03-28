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
                     SensorData prevData, SensorData currData, LidarScan prevScan, LidarScan currScan) {
    sensorData.yaw = 0;
    Pose n[SAMPLES];
    float weights[SAMPLES];
    float weightsSum = 0;
    double ms;
    auto prevYaw = prevData.yaw;
    auto currYaw = currData.yaw;
    std::clock_t a = std::clock();
    for (int i = 0; i < SAMPLES; i++) {
        auto diff = LidarScan::calcOffset(prevScan, prevYaw, currScan, currYaw);
        n[i] = Pose(input[i], measuredAccelForward, measuredAccelLateral, 0);
        n[i] = Pose(n[i], std::get<0>(diff), std::get<1>(diff));
        weights[i] = 10000/static_cast<float>(prevScan.raytrace(n[i])); //TODO who knows
        n[i].probability = weights[i];
        weightsSum += weights[i];
    }
    ms = (std::clock() - a) / (double) (CLOCKS_PER_SEC * 2.7 / 1000);
    std::cout << "Total raytrace time: " << ms << "ms" << std::endl;
    std::clock_t start = std::clock();
    std::vector<int> poseRegen = std::vector<int>();
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
    float probSum = 0;
    std::vector<float> probs;
    for (int i = 0; i < SAMPLES; ++i) {
        probSum += input[i].probability;
        probs.push_back(input[i].probability);
    }
    for (int i = 0; i < SAMPLES; ++i) {
        total_pose = input[i] + total_pose;
    }
    Pose average_pose = total_pose/SAMPLES;
    return average_pose;
}