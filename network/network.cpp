#include <fstream>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgproc.hpp>
#include <set>
#include <unordered_map>
#include <iostream>
#include "network.hpp"

Target::Target(float xCenter, float yCenter, float width, float height, float confidence)
        : xCenter(xCenter), yCenter(yCenter), width(width), height(height), confidence(confidence) {};

Network::Network(cv::String classNames, cv::String config, cv::String model) {
    frameMutex.lock();
    this->analyzedFrame = 0;
    this->network = new Detector(config, model);
    std::ifstream classNamesFile(classNames.c_str());
    if (classNamesFile.is_open()) {
        std::string className;
        while (std::getline(classNamesFile, className))
            this->classNames.emplace_back(className);
    }
    network->nms = 0.02;
    frameMutex.unlock();
}

Network::Network(cv::String classNames, cv::String config, cv::String model, cv::String save, double capWidth,
                 double capHeight) {
    network = new Detector(config, model);
    std::ifstream classNamesFile(classNames.c_str());
    if (classNamesFile.is_open()) {
        std::string className;
        while (std::getline(classNamesFile, className))
            this->classNames.emplace_back(className);
    }
    if (!save.empty()) {
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

std::vector<bbox_t>
Network::tracking_id(std::vector<bbox_t> cur_bbox_vec, bool const change_history, int const frames_story,
                     int const max_dist) {
    return network->tracking_id(cur_bbox_vec, change_history, frames_story, max_dist);
}

void Network::update(cv::Mat frameUpdate, int frameCounter) {
    frameMutex.lock();
    this->frame = frameUpdate.clone();
    if (currentlyAnalyzing) {
        skippedFrames.push(frameUpdate.clone());
    }
    this->frameCounter = frameCounter;
    frameMutex.unlock();
}

void Network::run(std::function<cv::Mat()> frameFunc,
                  std::unordered_map<std::string, std::function<void(std::vector<bbox_t>)>> targetMap) {
    std::shared_ptr<image_t> det_image;
    std::unordered_map<std::string, std::vector<bbox_t>> targetMapInter;
    while (true) {
        targetMapInter = {};
        frameMutex.lock();
        int frameID = frameCounter;
        cv::Mat annotated = frame.clone();
        frameMutex.unlock();
        if (frame.empty()) {
            if (this->saveWriter.isOpened()) {
                std::printf("Image was empty. Goodbye.\n");
                saveWriter.release();
                break;
            }
            continue;
        }
        if (frame.cols == 0) {
            continue;
        }
        if (frame.channels() == 4) {
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
        }
        currentlyAnalyzing = true;
        std::vector<bbox_t> result_vec = network->detect(frame);
        std::printf("result size: %d\n", result_vec.size());
        this->draw_boxes(annotated, result_vec);
        for (auto &item : result_vec) {
            targetMapInter[classNames[item.obj_id]].emplace_back(item);
            this->show_console_result(item);
        }
        this->frameMutex.lock();
        this->annotatedFrame = annotated;
        std::printf("just analyzed frame %d, current frame is %d\n", analyzedFrame, frameCounter);
        for (auto pair : targetMap) {
//            if(!targetMapInter[pair.first].empty())
            pair.second(targetMapInter[pair.first]);
        }
        currentlyAnalyzing = false;
        while (skippedFrames.size() > 1)
            skippedFrames.pop();

        this->analyzedFrame = frameID;
        this->frameMutex.unlock();
        if (this->saveWriter.isOpened()) {
            this->saveWriter.write(annotatedFrame);
        }
    }
}

cv::Mat Network::getAnnotatedFrame() {
    return annotatedFrame;
}

cv::Mat Network::getFrame() {
    return frame;
}