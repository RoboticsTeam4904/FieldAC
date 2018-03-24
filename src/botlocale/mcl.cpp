#include "mcl.hpp"
#include <tuple>
#include <cmath>
#include <vector>

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

Segment::Segment(int xi, int yi, int xf, int yf) {
    this->start = std::tuple<int, int>(xi, yi);
    this->end = std::tuple<int, int>(xf, yf);
}

Segment::Segment(std::tuple<int, int> start, std::tuple<int, int> end) {
    this->start = start;
    this->end = end;
}

Segment Segment::rotate(int anchor_x, int anchor_y, float angle) {
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