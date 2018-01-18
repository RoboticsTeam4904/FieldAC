#include "cubetrack.hpp"

#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>
#include <fstream>
#include <thread>

namespace ObjectTracking {

    CubeTracker::CubeTracker(cv::String config, cv::String model, cv::String classNames) {
        darknet = cv::dnn::readNetFromDarknet(config, model);
        std::ifstream classNamesFile(classNames.c_str());
        if(classNamesFile.is_open()) {
            std::string className;
            while(std::getline(classNamesFile, className))
                this->classNames.emplace_back(className);
        }
    }

    CubeTracker::CubeTracker(cv::String config, cv::String model, cv::String classNames, cv::String save, double capWidth, double capHeight) {
        darknet = cv::dnn::readNetFromDarknet(config, model);
        std::ifstream classNamesFile(classNames.c_str());
        if(classNamesFile.is_open()) {
            std::string className;
            while(std::getline(classNamesFile, className))
                this->classNames.emplace_back(className);
        }
        if(!save.empty()) {
            saveWriter.open(save, CV_FOURCC('M', 'J', 'P', 'G'), 3, cv::Size((int) capWidth, (int) capHeight), 1);
        }
    }

    cv::Mat CubeTracker::getFrame() {
        return frame.clone();
    }

    void CubeTracker::run(std::function<cv::Mat ()> frameFunc) {
        while(true) {
            cv::Mat frame = frameFunc();
            if(frame.empty()) {
                std::printf("Image was empty. Goodbye.\n");
                break;
            }
            if(frame.cols == 0) {
                continue;
            }
            if (frame.channels() == 4) {
                cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
            }
            cv::Mat inputBlob = cv::dnn::blobFromImage(frame, 1 / 255.F, cv::Size(416, 416), cv::Scalar(), true, false);
            std::printf("Building blob from image...\n");

            darknet.setInput(inputBlob, "data");

            cv::Mat detectionMat = darknet.forward("detection_out");

            std::vector<double> layersTimings;
            double tickFreq = cv::getTickFrequency();
            double timeMs = darknet.getPerfProfile(layersTimings) / tickFreq * 1000;
            cv::putText(frame, cv::format("FPS: %.2f ; time: %.2f ms", 1000.f / timeMs, timeMs),
                cv::Point(20, 20), 0, 0.5, cv::Scalar(0, 0, 255));
            std::printf("FPS: %.2f ; time: %.2f ms ; ", 1000.f / timeMs, timeMs);

            float confidenceThreshold = 0.25f;
            for (int i = 0; i < detectionMat.rows; i++) {

                const int probabilityIndex = 5;
                const int probabilitySize = detectionMat.cols - probabilityIndex;
                float *probArrayPtr = &detectionMat.at<float>(i, probabilityIndex);

                std::size_t objectClass = std::max_element(probArrayPtr, probArrayPtr + probabilitySize) - probArrayPtr;
                float confidence = detectionMat.at<float>(i, (int)objectClass + probabilityIndex);
                if(confidence > confidenceThreshold) {
                    float xCenter = detectionMat.at<float>(i, 0) * frame.cols;
                    float yCenter = detectionMat.at<float>(i, 1) * frame.rows;
                    float width = detectionMat.at<float>(i, 2) * frame.cols;
                    float height = detectionMat.at<float>(i, 3) * frame.rows;
                    cv::Point p1(cvRound(xCenter - width / 2), cvRound(yCenter - height / 2));
                    cv::Point p2(cvRound(xCenter + width / 2), cvRound(yCenter + height / 2));
                    cv::Rect object(p1, p2);
                    cv::Scalar objectRoiColor(0, 255, 0);
                    cv::rectangle(frame, object, objectRoiColor);
                    cv::String className = objectClass<classNames.size() ? classNames[objectClass] : cv::format("unknown(%d)", objectClass);
                    cv::String label = cv::format("%s: %.2f", className.c_str(), confidence);
                    std::printf("%s\n", className.c_str());
                    int baseLine = 0;
                    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
                    cv::rectangle(frame, cv::Rect(p1, cv::Size(labelSize.width, labelSize.height + baseLine)), objectRoiColor, CV_FILLED);
                    cv::putText(frame, label, p1 + cv::Point(0, labelSize.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0));
                }
            }
            std::printf("Iterated over %d rows", detectionMat.rows);
            if(saveWriter.isOpened()) {
                saveWriter.write(frame);
            }
            frameMutex.lock();
            this->frame = frame;
            frameMutex.unlock();
        }
    }

    CubeTracker::CubeTracker() = default;
}
