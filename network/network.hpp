#ifndef PROV_NETWORK_H
#define PROV_NETWORK_H

#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp>
#include <set>
#include <unordered_map>
#define OPENCV 1
#include "../darknet/yolo_v2_class.hpp"
#include "target.hpp"

class Network {
public:
    Network(cv::String classNames, cv::String config, cv::String model);
    Network(cv::String classNames, cv::String config, cv::String model, cv::String save, double capWidth, double capHeight);


    void run(std::function<cv::Mat ()> frameFunc, std::unordered_map<std::string, std::function<void(std::vector<Target>)>>);
    cv::Mat getAnnotatedFrame();

    void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, std::vector<std::string> obj_names, unsigned int wait_msec = 0, int current_det_fps = -1, int current_cap_fps = -1);
    void show_console_result(std::vector<bbox_t> const result_vec, std::vector<std::string> const obj_names);
private:
    Detector* network;
    std::vector<std::string> classNames;
    cv::VideoWriter saveWriter;

    cv::Mat annotatedFrame;
    mutable std::mutex frameMutex;
};

#endif