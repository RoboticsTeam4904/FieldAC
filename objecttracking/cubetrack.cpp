#include "cubetrack.hpp"
#include <thread>

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
        this->track_optflow_queue.push(frameUpdate.clone());
    }

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    void CubeTracker::run() {
        std::printf("Running CubeTracker in here!\n");
        cv::Mat frame;
        std::vector<bbox_t> opticalFlowBox;
        std::vector<cv::Point2f> features_prev, features_next;
        std::vector<cv::Point2f> good_features_prev, good_features_next;

        const int max_count = 1000;
        bool first = true;

        while(true) {
            if (this->track_optflow_queue.empty()) {
                continue;
            }
            if(first || this->recalc) {
                cv::Mat gray(this->track_optflow_queue.front().size(), CV_8UC1);
                cv::cvtColor(this->track_optflow_queue.front(), gray, CV_BGR2GRAY, 1);
                cv::goodFeaturesToTrack(gray, // the image
                                        features_next,   // the output detected features
                                        max_count,  // the maximum number of features
                                        0.02,     // quality level
                                        10
                );
                first = false;
                recalc = false;
            }

            while (this->track_optflow_queue.size() > 1) {
                cv::Mat current_frame(this->track_optflow_queue.front().size(), CV_8UC1); // Initialize greyscale current frame mat
                cv::cvtColor(this->track_optflow_queue.front(), current_frame, CV_BGR2GRAY, 1); // Convert front of queue to greyscale and put it in current_frame
                this->track_optflow_queue.pop();

                cv::Mat next_frame(this->track_optflow_queue.front().size(), CV_8UC1);
                cv::cvtColor(this->track_optflow_queue.front(), next_frame, CV_BGR2GRAY, 1); // Convert front of queue to greyscale and put it in next_frame
                this->track_optflow_queue.pop();

                features_prev = features_next;
                std::vector<unsigned char> status;
                std::vector<float> err;

                cv::calcOpticalFlowPyrLK(
                        current_frame, next_frame, // 2 consecutive images
                        features_prev, // input point positions in first im
                        features_next, // output point positions in the 2nd
                        status,    // tracking success
                        err,     // tracking error
                        cv::Size(21, 21),
                        3,
                        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
                        cv::OPTFLOW_USE_INITIAL_FLOW
                );

                optflowFrame = current_frame.clone();
                this->optflowFrameLast = next_frame.clone();

                size_t i, j;
                for (i = j = 0; i < features_next.size(); i++) {
                    if(!status[i]) {
                        continue;
                    }
                    features_next[j++] = features_next[i];
                    cv::circle(optflowFrame, features_next[i], 3, cv::Scalar(0,255,0), -1, 8);
                    if (features_next[i] != features_prev[i]) {
                        cv::line(optflowFrame, features_prev[i], features_next[i], cv::Scalar( 0, 255, 0 ), 1);
                    }
                }

                features_next.resize(j);
                this->draw_boxes(optflowFrame, opticalFlowBox);
            }

        }


        // TODO: C++ thrashed me and wouldn't let me check if the past targets were equal.
        // TODO: Hopefully I don't prank myself and someone else in the future fixes this
        // TODO: In all honestly it will probably be me though.
    }
#pragma clang diagnostic pop

    void CubeTracker::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec) {
        for (auto &i : result_vec) {
            cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), cv::Scalar(50, 200, 50), 3);
        }
    }

}
