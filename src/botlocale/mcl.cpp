#include "mcl.hpp"
#include "lidar.hpp"
#include "../field.hpp"
#include <tuple>
#include <cmath>
#include <vector>
#include <iostream>

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

Pose *BotLocale::init() {
    static Pose poses[SAMPLES];
    for (auto &pose : poses) {
        pose.seed();
    }
    return poses;
}

Pose *BotLocale::step(Pose input[1000], SensorData prevData, SensorData currData, std::deque<LidarScan> scans) {
    Pose n[SAMPLES];
    float weights[SAMPLES];
    float weightsSum = 0;
    double ms;
    auto prevYaw = prevData.yaw;
    auto currYaw = currData.yaw;
    std::clock_t a = std::clock();
    auto diff = LidarScan::calcOffset(scans);
    std::cout << std::get<0>(diff) << std::endl;
    for (int i = 0; i < SAMPLES; i++) {
        n[i] = Pose(input[i], std::get<0>(diff), std::get<1>(diff), currData);
        weights[i] = static_cast<float>(scans.back().raytrace(input[i]));
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
    std::cout << "Average MCL time: " << ms / SAMPLES << "ms" << std::endl;
    return input;
}

Pose BotLocale::get_best_pose(Pose input[SAMPLES], LidarScan scan) {
//    return input[(int)RAND*SAMPLES];
    Pose total_pose;
    float probSum = 0;
    std::vector<float> probs;
    Pose best_pose;
    best_pose.probability = 0;
    for (int i = 0; i < SAMPLES; ++i) {
        if (input[i].probability > best_pose.probability) {
            best_pose = input[i];
        }
        probSum += input[i].probability;
        probs.push_back(input[i].probability);
    }
    for (int i = 0; i < SAMPLES; ++i) {
        total_pose = input[i] + total_pose;
    }
    Pose average_pose = total_pose / SAMPLES;
    std::cout << "best score :" << best_pose.probability << std::endl;
    return average_pose;
}