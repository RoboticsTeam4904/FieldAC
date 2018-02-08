#include <fstream>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgproc.hpp>
#include <set>
#include <unordered_map>
#include "network.hpp"

Target::Target(float xCenter, float yCenter, float width, float height, float confidence)
        : xCenter(xCenter), yCenter(yCenter), width(width), height(height), confidence(confidence) {};

Network::Network(cv::String classNames, cv::String config, cv::String model) {
    network = new Detector(config, model);

    std::ifstream classNamesFile(classNames.c_str());
    if(classNamesFile.is_open()) {
        std::string className;
        while(std::getline(classNamesFile, className))
            this->classNames.emplace_back(className);
    }
    network->nms = 0.02;
}

Network::Network(cv::String classNames, cv::String config, cv::String model, cv::String save, double capWidth, double capHeight) {
    network = new Detector(config, model);
    std::ifstream classNamesFile(classNames.c_str());
    if(classNamesFile.is_open()) {
        std::string className;
        while(std::getline(classNamesFile, className))
            this->classNames.emplace_back(className);
    }
    if(!save.empty()) {
        saveWriter.open(save, CV_FOURCC('X', '2', '6', '4'), 3, cv::Size((int) capWidth, (int) capHeight), 1);
    }
    network->nms = 0.02;
}

void Network::show_console_result(std::vector<bbox_t> const result_vec) {
    for (auto &i : result_vec) {
        this->show_console_result(i);
    }
}

void Network::show_console_result(bbox_t const result) {
    std::cout << "obj_id = " << result.obj_id << ",  x = " << result.x << ", y = " << result.y
              << ", w = " << result.w << ", h = " << result.h
              << ", prob = " << result.prob << std::endl;
}

void Network::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec) {
    for (auto &i : result_vec) {
        cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), cv::Scalar(50, 200, 50), 3);
    }
}

void Network::run(std::function<cv::Mat ()> frameFunc,
                  std::unordered_map<std::string, std::function<void(std::vector<Target>)>> targetMap) {
    std::shared_ptr<image_t> det_image;
    std::unordered_map<std::string, std::vector<Target>> targetMapInter;
    while(true) {
        targetMapInter = {};
        cv::Mat frame = frameFunc();
        cv::Mat annotated = frame.clone();
        if(frame.empty()) {
            std::printf("Image was empty. Goodbye.\n");
            saveWriter.release();
            break;
        }
        if(frame.cols == 0) {
            continue;
        }
        if (frame.channels() == 4) {
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
        }
        std::vector<bbox_t> result_vec = network->detect(frame);
        this->draw_boxes(annotated, result_vec);
        for(auto &item : result_vec) {
            targetMapInter[classNames[item.obj_id]].emplace_back(Target(item));
            this->show_console_result(item);
        }
        for(auto pair : targetMap) {
            pair.second(targetMapInter[pair.first]);
        }
        this->frameMutex.lock();
        this->annotatedFrame = annotated;
        this->frameMutex.unlock();
        if(this->saveWriter.isOpened()) {
            this->saveWriter.write(annotatedFrame);
        }
    }
}

cv::Mat Network::getAnnotatedFrame() {
    return annotatedFrame;
}