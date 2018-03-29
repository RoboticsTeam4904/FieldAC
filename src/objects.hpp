#ifndef OBJECTS_H
#define OBJECTS_H

#include <string>
#include <tuple>
#include <opencv2/core/matx.hpp>

struct SensorData {
    double leftEncoder;
    double rightEncoder;
    double accelX;
    double accelZ;
    double accelY;
    double yaw;
    bool operator==(const SensorData other) {
        return (leftEncoder == other.leftEncoder && rightEncoder == other.rightEncoder && accelX == other.accelX);
    }
};

struct Pose {
    Pose();
    Pose(const Pose prev, const cv::Vec2f offset, float yaw_diff, SensorData sensorData);
    Pose(const Pose prev, const float measuredAccelForward, const float measuredAccelLateral, const float measuredAccelYaw);
    float x;
    float y;
    float dx;
    float dy;
    float yaw;
    float rateYaw;
    float probability;
    float plausibility(const std::string sensorData);
    void seed();
    Pose& operator+(const Pose& other);
    Pose& operator/(const int& other);
    bool operator==(const Pose& other);
    Pose& operator*(const float& other);
};

struct Segment {
    Segment(double xi, double yi, double xf, double yf);
    Segment(std::tuple<double, double> start, std::tuple<double, double> end);
    Segment rotate(double, double, float);
    std::tuple<double, double> start;
    std::tuple<double, double> end;
};


#endif
