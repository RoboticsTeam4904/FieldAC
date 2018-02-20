#include "cubetrack.hpp"
#include <thread>
#include "extrapolate.hpp"

#ifndef OPENCV
#define OPENCV 1
#endif
#ifndef TRACK_OPTFLOW
#define TRACK_OPTFLOW 1
#endif
#ifdef GPU
#undef GPU
#endif

namespace ObjectTracking {
    CubeTracker::CubeTracker(Network& network) : network(network) {
        this->tracker_flow = new Tracker_optflow();
    }

    void CubeTracker::update(std::vector<bbox_t> targetsUpdate) {
        this->mutexTargets.lock();
        this->targetsLast = this->targets;
        this->targets = targetsUpdate;
        this->mutexTargets.unlock();
    }

    void CubeTracker::update(cv::Mat frameUpdate) {
        this->mutexFrame.lock();
        this->lastLastFrame = this->lastFrame.clone();
        this->lastFrame = frameUpdate;
        this->mutexFrame.unlock();
        this->track_optflow_queue.push(frameUpdate);
    }

    void CubeTracker::run() {
        std::printf("Running CubeTracker in here!\n");
        cv::Mat frame;
        float cur_time_extrapolate = 0, old_time_extrapolate = 0;
        std::vector<bbox_t> opticalFlowBox;
        while(true) {
            this->mutexFrame.lock();
            frame = this->lastFrame.clone();
            this->mutexFrame.unlock();
//            std::printf("Frame height: %d\n", frame.size().height);
            if (frame.empty()) {
                continue;
            }
            std::vector<cv::Point> features_prev, features_next;

            int max_count = 100;
            std::printf("suck\n");
            cv::Mat gray(this->lastFrame.size(), CV_8UC1);
            cv::cvtColor(this->track_optflow_queue.front(), gray, CV_BGR2GRAY, 1);
            cv::goodFeaturesToTrack(gray, // the image
                                    features_next,   // the output detected features
                                    max_count,  // the maximum number of features
                                    0.01,     // quality level
                                    10
            );
            std::printf("size of thingyyyyyyy: %d\n\n", features_next.size());
            for (auto &i : features_next) {
                std::printf(" - feature: (%d, %d)\n", i.x, i.y);
            }
            while (this->track_optflow_queue.size() > 1) {
                std::printf("i want to die please and thank you\n");
                cv::Mat current_frame(this->lastFrame.size(), CV_8UC1); // Initialize greyscale current frame mat
                cv::cvtColor(this->track_optflow_queue.front(), current_frame, CV_BGR2GRAY, 1); // Convert front of queue to greyscale and put it in current_frame

                this->track_optflow_queue.pop();

                cv::Mat next_frame(this->lastFrame.size(), CV_8UC1);
                cv::cvtColor(this->track_optflow_queue.front(), current_frame, CV_BGR2GRAY, 1); // Convert front of queue to greyscale and put it in next_frame

                features_prev = features_next;
                cv::Mat status;
                std::vector<cv::Point> err;
                std::printf("size of 1: %lu\nsize of 2:%lu\n", features_next.size(), features_prev.size());
                cv::calcOpticalFlowPyrLK(
                        current_frame, next_frame, // 2 consecutive images
                        features_prev, // input point positions in first im
                        features_next, // output point positions in the 2nd
                        status,    // tracking success
                        err      // tracking error
                );
                std::printf("Did optical flow\n");
                for (int i = 0; i < features_next.size(); i++) {
                    std::printf(" --- point (%d, %d) -> (%d, %d)", features_prev[i].x, features_prev[i].y, features_next[i].x, features_next[i].y);
                }
            }

            optflowFrame = frame.clone();
            this->draw_boxes(optflowFrame, opticalFlowBox);
        }


        // TODO: C++ thrashed me and wouldn't let me check if the past targets were equal.
        // TODO: Hopefully I don't prank myself and someone else in the future fixes this
        // TODO: In all honestly it will probably be me though.
    }

    void CubeTracker::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec) {
        for (auto &i : result_vec) {
            cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), cv::Scalar(50, 200, 50), 3);
        }
    }

}
