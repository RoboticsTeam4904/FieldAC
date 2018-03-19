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
#include <queue>
#include <mutex>
#include <condition_variable>

class Network {
public:
    Network(cv::String classNames, cv::String config, cv::String model);
    Network(cv::String classNames, cv::String config, cv::String model, cv::String save, double capWidth, double capHeight);

    void run(std::function<cv::Mat ()> frameFunc, std::unordered_map<std::string, std::function<void(std::vector<bbox_t>)>>);
    void update(cv::Mat frameUpdate, int frameCounter);
    cv::Mat getAnnotatedFrame();
    cv::Mat getFrame();

    void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec);
    void show_console_result(const std::vector<bbox_t> result_vec);
    void show_console_result(const bbox_t result);

    std::queue<cv::Mat> skippedFrames;

    std::vector<bbox_t> tracking_id(std::vector<bbox_t> cur_bbox_vec, const bool change_history = true, const int frames_story = 10, const int max_dist = 150);
    int analyzedFrame;
    mutable std::mutex frameMutex;
private:
    Detector* network;
    std::vector<std::string> classNames;
    cv::VideoWriter saveWriter;

    cv::Mat annotatedFrame;
    cv::Mat frame;
    int frameCounter;

    bool currentlyAnalyzing;
};

#endif