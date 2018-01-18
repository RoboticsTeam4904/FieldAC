#ifndef NETWORK_H
#define NETWORK_H

#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp>
#include <set>

struct Target {
    Target(float xCenter, float yCenter, float width, float height, float confidence);
    float xCenter;
    float yCenter;
    float width;
    float height;
    float confidence;
};

struct Network {
    Network(cv::String config, cv::String model, cv::String classNames);
    Network(cv::String config, cv::String model, cv::String classNames, cv::String save, double capWidth, double capHeight);

    cv::dnn::Net network;
    std::vector<cv::String> classNames;
    cv::VideoWriter saveWriter;

    cv::Mat annotatedFrame;
    mutable std::mutex frameMutex;

    void run(std::function<cv::Mat ()> frameFunc, std::map<std::string, std::function<void(cv::Mat, std::vector<Target>)>>);
    cv::Mat getAnnotatedFrame();
};

#endif
