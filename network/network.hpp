#ifndef FRC_NETWORK_H
#define FRC_NETWORK_H

#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/videoio.hpp>
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

class Network {
    public:
        Network(cv::String data, cv::String config, cv::String model);
        Network(cv::String data, cv::String config, cv::String model, cv::String save, double capWidth, double capHeight);

        ArapahoV2* network;
        cv::VideoWriter saveWriter;

        cv::Mat annotatedFrame;
        mutable std::mutex frameMutex;

        void run(std::function<cv::Mat ()> frameFunc, std::unordered_map<std::string, std::function<void(cv::Mat, std::vector<Target>)>>);
        cv::Mat getAnnotatedFrame();

        /**
         * Overloading the `=` operator.
         * We ensure that the copying of members happens in a locked state for
         * for both sides of the assignment, and more importantly that the locking
         * is safely released after the assignment is completed. .
         */
        Network& operator=(const Network& origin) {
            if (this != &origin) {
                std::lock(frameMutex, origin.frameMutex);
                std::lock_guard<std::mutex> lhs_lk(frameMutex, std::adopt_lock);
                std::lock_guard<std::mutex> rhs_lk(origin.frameMutex, std::adopt_lock);
                network = origin.network;
                saveWriter = origin.saveWriter;

                annotatedFrame = origin.annotatedFrame;
            }
            return *this;
        }
};

#endif