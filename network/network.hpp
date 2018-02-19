#ifndef PROV_NETWORK_H
#define PROV_NETWORK_H

#ifndef OPENCV
#define OPENCV 1
#endif
#ifndef TRACK_OPTFLOW
#define TRACK_OPTFLOW 1
#endif
#ifdef GPU
#undef GPU
#endif
#include "yolo_v2_class.hpp"
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp>
#include <set>
#include <unordered_map>
#include "target.hpp"
#include <cstdlib>

class Network {
public:
    Network(cv::String classNames, cv::String config, cv::String model);
    Network(cv::String classNames, cv::String config, cv::String model, cv::String save, double capWidth, double capHeight);

    void run(std::function<cv::Mat ()> frameFunc, std::unordered_map<std::string, std::function<void(std::vector<bbox_t>)>>);
    cv::Mat getAnnotatedFrame();

    void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec);
    void show_console_result(std::vector<bbox_t> const result_vec);
    void show_console_result(bbox_t const result);

    std::vector<bbox_t> tracking_id(std::vector<bbox_t> cur_bbox_vec, bool const change_history = true, int const frames_story = 10, int const max_dist = 150);
private:
    Detector* network;
    std::vector<std::string> classNames;
    cv::VideoWriter saveWriter;

    cv::Mat annotatedFrame;
    mutable std::mutex frameMutex;
};

#endif