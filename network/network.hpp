#ifndef FRC_NETWORK_H
#define FRC_NETWORK_H

#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp>
#include <set>
#include <unordered_map>
#include "../arapaho/arapaho.hpp"

struct Target {
    Target(float xCenter, float yCenter, float width, float height, float confidence);
    float xCenter;
    float yCenter;
    float width;
    float height;
    float confidence;
};

struct Network {
    Network(cv::String data, cv::String config, cv::String model);
    Network(cv::String data, cv::String config, cv::String model, cv::String save, double capWidth, double capHeight);

    ArapahoV2* network;
    std::vector<cv::String> classNames;
    cv::VideoWriter saveWriter;

    cv::Mat annotatedFrame;
    mutable std::mutex frameMutex;

    void run(std::function<cv::Mat ()> frameFunc, std::unordered_map<std::string, std::function<void(cv::Mat, std::vector<Target>)>>);
    cv::Mat getAnnotatedFrame();
};

#endif