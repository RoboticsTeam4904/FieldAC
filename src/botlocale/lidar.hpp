#ifndef PROV_LIDAR_HPP
#define PROV_LIDAR_HPP

#include <cstddef>
#include <rplidar.h>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include "../objects.hpp"
#include <vector>
#include <tuple>
#include <opencv/cv.h>
#include <queue>

#ifndef _countof
#define _countof(_Array) (int)((sizeof(_Array)) / (sizeof(_Array[0])))
#endif

using namespace rp::standalone;

class LidarScan {
public:
    std::tuple<float, float> measurements[360];
    int offset;
    float yaw;
    std::vector<double> confidence;
public:
    LidarScan();
    LidarScan(const LidarScan &other, int newOffset);

    LidarScan generateExpected(const Pose &pose);

    LidarScan getAtLocation(int xCm, int yCm);

    double raytrace(Pose);

    cv::Point2f *intersect_ray_with_segment(cv::Point2f origin, cv::Vec2f direction, Segment seg);

    static std::tuple<cv::Vec2f, float> calcOffset(std::deque<LidarScan> scans);

    double raytrace_visual(Pose robot_pose, cv::Mat &img);

    inline float getAtAngle(int angle) const {
        return std::get<1>(measurements[angle]);
    }

private:


    float calc(float amount, float x, float y, float t1);
};

class Lidar {
protected:
    _u32 baudrate;
    std::string path;
public:
    rplidar::RPlidarDriver *driver;
public:
    Lidar(std::string path, _u32 baudrate);

    void run(const bool *stop);

    void stop();

    bool checkHealth();

    LidarScan current_scan;
};

cv::Point2f tuple_to_point(std::tuple<double, double>);

#endif
