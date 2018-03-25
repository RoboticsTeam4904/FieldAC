#include "mcl.hpp"
#include <tuple>
#include <cmath>
#include <vector>

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

Segment::Segment(double xi, double yi, double xf, double yf) {
    this->start = std::tuple<double, double>(xi, yi);
    this->end = std::tuple<double, double>(xf, yf);
}

Segment::Segment(std::tuple<double, double> start, std::tuple<double, double> end) {
    this->start = start;
    this->end = end;
}

Segment Segment::rotate(double anchor_x, double anchor_y, float angle) {
    int x1 = std::get<0>(this->start);
    int x2 = std::get<0>(this->end);
    int y1 = std::get<1>(this->start);
    int y2 = std::get<1>(this->end);
    auto x1_p = static_cast<int>(((x1 - anchor_x) * cos(angle) + (y1 - anchor_y) * sin(angle)) + anchor_x);
    auto x2_p = static_cast<int>(((x2 - anchor_x) * cos(angle) + (y2 - anchor_y) * sin(angle)) + anchor_x);
    auto y1_p = static_cast<int>((-(x1 - anchor_x) * sin(angle) + (y1 - anchor_y) * cos(angle)) + anchor_y);
    auto y2_p = static_cast<int>((-(x2 - anchor_x) * sin(angle) + (y2 - anchor_y) * cos(angle)) + anchor_y);
    return Segment(x1_p, y1_p, x2_p, y2_p);
}

Pose *BotLocale::init() {
    static Pose poses[SAMPLES];
    for (auto &pose : poses) {
        pose.seed();
    }
    return poses;
}

void BotLocale::step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelLateral,
                     const float measuredAccelYaw, std::string sensorData) {
    Pose n[SAMPLES];
    float weights[SAMPLES];
    float weightsSum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        n[i] = Pose(input[i], measuredAccelForward, measuredAccelLateral, measuredAccelYaw);
        weights[i] = n[i].plausibility(sensorData);
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
}

Pose BotLocale::get_best_pose(Pose input[SAMPLES]) {
    Pose total_pose;
    for (int i = 0; i < SAMPLES; ++i) {
        total_pose = input[i] + total_pose;
    }
    Pose average_pose = total_pose / SAMPLES;
    return average_pose;
}