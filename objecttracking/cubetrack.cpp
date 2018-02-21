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
        this->lastFrame.copyTo(this->lastLastFrame);
        this->lastFrame = frameUpdate;
        this->mutexFrame.unlock();
        this->track_optflow_queue.push(frameUpdate);
    }

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    void CubeTracker::run() {
        std::printf("Running CubeTracker in here!\n");
        cv::Mat frame;
        bool findNewFeatures = true;
        std::vector<bbox_t> opticalFlowBox;
        std::vector<cv::Point2f> features_prev, features_next;

        int daysSinceAccident = 0;
        while(true) {

            if (this->track_optflow_queue.empty()) {
                continue;
            }
//            this->mutexFrame.lock();
//            frame = this->lastFrame.clone();
//            this->mutexFrame.unlock();
//            std::printf("Frame height: %d\n", frame.size().height);
//            if (frame.empty()) {
//                continue;
//            }


            int max_count = 10000;
            cv::Mat gray(this->track_optflow_queue.front().size(), CV_8UC1);
            cv::Mat originalImage = this->track_optflow_queue.front();
            cv::cvtColor(this->track_optflow_queue.front(), gray, CV_BGR2GRAY, 1);
            if (true) {
                features_prev.clear();
                features_next.clear();
                findNewFeatures = false;
                cv::goodFeaturesToTrack(gray, // the image
                                        features_next,   // the output detected features
                                        max_count,  // the maximum number of features
                                        0.01,     // quality level
                                        10
                );
                features_prev = features_next;
                std::printf("FOUND N E W  POINTS         - - - - - \n");
            }

            while (this->track_optflow_queue.size() > 1) {
                std::printf("The queue is shit: %lu\n", this->track_optflow_queue.size());

                cv::Mat current_frame(this->lastFrame.size(), CV_8UC1); // Initialize greyscale current frame mat
                cv::cvtColor(this->track_optflow_queue.front(), current_frame, CV_BGR2GRAY, 1); // Convert front of queue to greyscale and put it in current_frame
                this->track_optflow_queue.pop();

                cv::Mat next_frame(this->lastFrame.size(), CV_8UC1);
                cv::cvtColor(this->track_optflow_queue.front(), current_frame, CV_BGR2GRAY, 1); // Convert front of queue to greyscale and put it in next_frame
                this->track_optflow_queue.pop();

                features_prev = features_next;
                std::vector<unsigned char> status;
                std::vector<float> err;
                std::printf("are they equal? %d\n", cv::countNonZero(current_frame != next_frame) == 0);
                cv::calcOpticalFlowPyrLK(
                        current_frame, next_frame, // 2 consecutive images
                        features_prev, // input point positions in first im
                        features_next, // output point positions in the 2nd
                        status,    // tracking success
                        err     // tracking error
                );
                optflowFrame = originalImage.clone();
                bool anychanged = false;
                for (int i = 0; i < features_next.size(); i++) {
                    if (features_next[i] != features_prev[i]) {
                        cv::line(optflowFrame, features_prev[i], features_next[i], cv::Scalar( 0, 255, 0 ), 1);
                        std::printf(" --- point (%lf, %lf) -> (%lf, %lf)\n", features_prev[i].x, features_prev[i].y, features_next[i].x, features_next[i].y);
                        anychanged = true;
                    }
                }
                if(!anychanged) {
                    daysSinceAccident++;
                }
                if(daysSinceAccident == 100) {
                    findNewFeatures = true;
                    daysSinceAccident = 0;
                }
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
