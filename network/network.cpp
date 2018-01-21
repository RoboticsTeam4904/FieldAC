#include <fstream>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgproc.hpp>
#include <set>
#include <unordered_map>
#include "network.hpp"

#define MAX_OBJECTS_PER_FRAME (100)

Target::Target(float xCenter, float yCenter, float width, float height, float confidence)
        : xCenter(xCenter), yCenter(yCenter), width(width), height(height), confidence(confidence) {};

Network::Network(cv::String data, cv::String config, cv::String model) {
    ArapahoV2Params params;
    params.datacfg = const_cast<char *>(data.c_str());
    params.cfgfile = const_cast<char *>(config.c_str());
    params.weightfile = const_cast<char *>(model.c_str());
    params.nms = 0.4;
    params.maxClasses = 2;

    network = new ArapahoV2();
    if(!network) {
        std::printf("ERROR: Could not initialize ArapahoV2 instance\n");
    }
    int expectedW = 0, expectedH = 0;

    if(!network->Setup(params, expectedW, expectedH)) {
        std::printf("ERROR: Could not setup network instance\n");
    }
}

Network::Network(cv::String data, cv::String config, cv::String model, cv::String save, double capWidth, double capHeight) {
    ArapahoV2Params params;
    params.datacfg = const_cast<char *>(data.c_str());
    params.cfgfile = const_cast<char *>(config.c_str());
    params.weightfile = const_cast<char *>(model.c_str());
    params.nms = 0.4;
    params.maxClasses = 2;

    network = new ArapahoV2();
    if(!network) {
        std::printf("ERROR: Could not initialize ArapahoV2 instance\n");
    }
    int expectedW = 0, expectedH = 0;
    if(!network->Setup(params, expectedW, expectedH)) {
        std::printf("ERROR: Could not setup network instance\n");
    }
    if(!save.empty()) {
        saveWriter.open(save, CV_FOURCC('M', 'J', 'P', 'G'), 3, cv::Size((int) capWidth, (int) capHeight), 1);
    }
}

void Network::run(std::function<cv::Mat ()> frameFunc,
                  std::unordered_map<std::string, std::function<void(cv::Mat, std::vector<Target>)>> targetMap) {
    box* boxes = 0;
    std::string* labels;
    while(true) {
        cv::Mat frame = frameFunc();
        cv::Mat annotated = frame.clone();
        if(frame.empty()) {
            std::printf("Image was empty. Exiting.\n");
            break;
        }
        if(frame.cols == 0) {
            continue;
        }
        if(frame.channels() == 4) {
            cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
        }
        int numObjects = 0;
        auto detectionStartTime = std::chrono::system_clock::now();
        network->Detect(frame, 0.24, 0.5, numObjects);
        std::chrono::duration<double> detectionTime = (std::chrono::system_clock::now() - detectionStartTime);
        std::printf("Detected [%d] objects in [%f] seconds\n", numObjects, detectionTime);
        if(numObjects > 0 && numObjects < MAX_OBJECTS_PER_FRAME) {
            boxes = new box[numObjects];
            labels = new std::string[numObjects];
            network->GetBoxes(boxes, labels, numObjects);
            std::map<std::string, std::vector<Target>> targets;
            std::map<std::string, std::vector<Target>>::iterator targetsIter;
            for (int i = 0; i < numObjects; i++) {
                Target target(boxes[i].x, boxes[i].y, boxes[i].w, boxes[i].h, 1);
                targetsIter = targets.find(labels[i]);
                if (targetsIter != targets.end()) {
                    targetsIter->second.push_back(target);
                } else {
                    std::vector<Target> value;
                    value.push_back(target);
                    targets[labels[i]] = value;
                }
                cv::Point p1(cvRound(boxes[i].x - boxes[i].w / 2), cvRound(boxes[i].y - boxes[i].h / 2));
                cv::Point p2(cvRound(boxes[i].x + boxes[i].w / 2), cvRound(boxes[i].y + boxes[i].h / 2));
                cv::Rect object(p1, p2);
                cv::Scalar objectRoiColor(0, 255, 0);
                cv::rectangle(annotated, object, objectRoiColor);
                auto tempLabel = labels[i];
                cv::String label = cv::format("%s: %.2f", tempLabel.c_str(), 1);
                int baseLine = 0;
                cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                cv::rectangle(annotated, cv::Rect(p1, cv::Size(labelSize.width, labelSize.height + baseLine)),
                              objectRoiColor, CV_FILLED);
                cv::putText(annotated, label, p1 + cv::Point(0, labelSize.height), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 0, 0));
            }
            for (const auto &iter : targetMap) {
                targetsIter = targets.find(iter.first);
                if (targetsIter != targets.end()) {
                    iter.second(frame, targetsIter->second);
                } else {
                    iter.second(frame, std::vector<Target>());
                }
            }
            if (saveWriter.isOpened()) {
                saveWriter.write(annotated);
            }
            frameMutex.lock();
            this->annotatedFrame = annotated;
            frameMutex.unlock();
        }
    }
}

cv::Mat Network::getAnnotatedFrame() {
//    std::lock_guard<std::mutex> lkg(this->frameMutex);
    return annotatedFrame;
}