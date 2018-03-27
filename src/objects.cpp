#include "objects.hpp"
#include <math.h>
#include <cmath>

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

#define ZRAND (RAND - 0.5)

#define VELOCITY_NOISE 0.0001

#define YAW_RATE_NOISE 0.0001

Pose::Pose() = default;

Pose::Pose(const Pose prev, const float measuredAccelForward, const float measuredAccelLateral,
           const float measuredAccelYaw) {
    x = prev.x + prev.dx;
    y = prev.y + prev.dy;

    dx = static_cast<float>(prev.dx + measuredAccelForward * cos(prev.yaw) - measuredAccelLateral * sin(prev.yaw) +
            (ZRAND * VELOCITY_NOISE));
    dy = static_cast<float>(prev.dy + measuredAccelForward * sin(prev.yaw) + measuredAccelLateral * cos(prev.yaw) +
            (ZRAND * VELOCITY_NOISE));

    yaw = prev.yaw + prev.rateYaw;
    rateYaw = static_cast<float>(prev.rateYaw + measuredAccelYaw + (ZRAND * YAW_RATE_NOISE));
}

void Pose::seed() {
    x = RAND*849;
    y = RAND*1700;
    dx = 0;
    dy = 0;
    rateYaw = 0;
    yaw = 0;
}

float Pose::plausibility(const std::string sensorData) {
    if (x < -10 || x > 10 || y < -10 || y > 10) {
        return 0.1;
    }
    return 1;
}

Pose &Pose::operator+(const Pose &other) {
    static auto tmp = Pose();
    tmp.x = this->x + other.x;
    tmp.y = this->y + other.y;
    tmp.yaw = this->yaw + other.yaw;
    tmp.dx = this->dx + other.dx;
    tmp.dy = this->dy + other.dy;
    tmp.rateYaw = this->rateYaw + other.rateYaw;
    return tmp;
}

Pose &Pose::operator/(const int &other) {
    static auto tmp = Pose();
    tmp.x = this->x / other;
    tmp.y = this->y / other;
    tmp.yaw = this->yaw / other;
    while (tmp.yaw > (M_PI*2)) {
        tmp.yaw -= M_PI*2;
    }
    tmp.dx = this->dx / other;
    tmp.dy = this->dy / other;
    tmp.rateYaw = this->rateYaw / other;
    return tmp;
}

bool Pose::operator==(const Pose &other) {
    return (std::abs(this->x - other.x) < 3 && std::abs(this->y - other.y) < 3); // TODO tune this factor
}

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