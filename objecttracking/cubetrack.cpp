#include "cubetrack.hpp"
#include <thread>
#include <sys/time.h>

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
    CubeTracker::CubeTracker(Network &network) : network(network) {
        this->tracker_flow = new Tracker_optflow();
    }

    void CubeTracker::update(std::vector<bbox_t> targetsUpdate) {
        this->targetsLast = this->targets;
        this->targets = targetsUpdate;
        // propagate through old frames
        std::printf("old - ");
        network.show_console_result(targetsUpdate);
        timeval old;
        gettimeofday(&old, nullptr);
        this->mutexTargets.lock();
        this->targets = this->extrapolate_bbox_through_queue(targetsUpdate, network.skippedFrames);
        timeval newt;
        std::printf("new - ");
        network.show_console_result(this->targets);
        gettimeofday(&newt, nullptr);
        std::printf("interpolated through skipped frames, took %d \n", newt.tv_usec - old.tv_usec);
        this->mutexTargets.unlock();
        newTargets = true;
    }

    void CubeTracker::update(cv::Mat frameUpdate, int frameCounter) {
        this->frameCounter = frameCounter;
        if (this->targets.empty()) {
            return;
        }
        if (frameCounter % 3) {
            this->track_optflow_queue.push(frameUpdate.clone());
        }
    }

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

    void CubeTracker::run() {
        std::printf("Running CubeTracker in here!\n");
        cv::Mat frame;
        std::vector<bbox_t> opticalFlowBox;

        std::vector<cv::Point2f> features_prev, features_next;

        const int max_count = 1000;
        const int min_count = 500;
        bool first = true;

        std::vector<bbox_t> descaledDetections;

        while (true) {
            if (this->track_optflow_queue.empty() || this->targets.empty()) {
                continue;
            }
            if (first || this->recalc) {
                mutexTargets.lock();
                descaledDetections = this->targets;
                if (!descaledDetections.empty()) {
                    features_next.clear();
                }
                for (auto &i : descaledDetections) {
                    float centerX = i.x + (i.w / 2.0F);
                    float centerY = i.y + (i.h / 2.0F);
                    cv::Point2f centerPoint;
                    centerPoint.x = centerX;
                    centerPoint.y = centerY;
                    features_next.push_back(centerPoint);
                }
                mutexTargets.unlock();



//                    cv::Mat gray(this->track_optflow_queue.front().size(), CV_8UC1);
//                    cv::cvtColor(this->track_optflow_queue.front(), gray, CV_BGR2GRAY, 1);
//                    cv::goodFeaturesToTrack(gray, // the image
//                                            features_next,   // the output detected features
//                                            max_count,  // the maximum number of features
//                                            0.02,     // quality level
//                                            1
//                    );
                first = false;
                recalc = false;

            }


            /**
             * Realistically, this won't be needed once we implement Cube Flow.
             * This is here to make it more use-able for testing using an arbitrary object.
             */

//            if (features_next.size() <= min_count) {
//                std::printf("Could not track features... Recalculating: %d\n", static_cast<int>(max_count - features_next.size()));
//                cv::Mat gray(this->track_optflow_queue.front().size(), CV_8UC1);
//                cv::cvtColor(this->track_optflow_queue.front(), gray, CV_BGR2GRAY, 1);
//                std::vector<cv::Point2f> recalculation;
//                cv::goodFeaturesToTrack(gray,
//                                        recalculation,
//                                        static_cast<int>(max_count - features_next.size()),
//                                        0.02,
//                                        1);
//                features_next.insert(features_next.begin(), recalculation.begin(), recalculation.end());
//            }
                while (this->track_optflow_queue.size() > 1) {
                    cv::Mat current_frame(this->track_optflow_queue.front().size(),
                                          CV_8UC1); // Initialize greyscale current frame mat
                    cv::cvtColor(this->track_optflow_queue.front(), current_frame, CV_BGR2GRAY,
                                 1); // Convert front of queue to greyscale and put it in current_frame
                    this->track_optflow_queue.pop();

                    cv::Mat next_frame(this->track_optflow_queue.front().size(), CV_8UC1);
                    cv::cvtColor(this->track_optflow_queue.front(), next_frame, CV_BGR2GRAY,
                                 1); // Convert front of queue to greyscale and put it in next_frame
                    this->track_optflow_queue.pop();

                    features_prev = features_next;
                    std::vector<unsigned char> status;
                    std::vector<float> err;

                    if (features_next.empty()) {
                        this->recalc = true;
                        features_next = features_prev;
                         // we lost it so assume it didn't move much?
                        continue;
                    }

                    cv::calcOpticalFlowPyrLK(
                            current_frame, next_frame, // 2 consecutive images
                            features_prev, // input point positions in first im
                            features_next, // output point positions in the 2nd
                            status,    // tracking success
                            err,     // tracking error
                            cv::Size(15, 15),
                            3,
                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1),
                            cv::OPTFLOW_USE_INITIAL_FLOW
                    );

                    optflowFrame = current_frame.clone();
                    this->optflowFrameLast = next_frame.clone();

                    if (opticalFlowBox.empty() || newTargets) {
                        opticalFlowBox.clear();
                        opticalFlowBox = this->targets;
                        recalc = true;
                        newTargets = false;
                    }

                    for (size_t i = 0; i < features_next.size(); i++) {
                        cv::Point2f point_next = features_next.at(i);
                        cv::Point2f point_prev = features_prev.at(i);
                        mutexTargets.lock();
                        opticalFlowBox.at(i).x = static_cast<unsigned int>((point_next.x - (this->targets.at(i).w / 2)));
                        opticalFlowBox.at(i).y = static_cast<unsigned int>((point_next.y - (this->targets.at(i).h / 2)));
                        mutexTargets.unlock();
                    }

                    size_t i, j;
                    for (i = j = 0; i < features_next.size(); i++) {
                        // Check if the feature was successfully tracked, if not then skip.
                        if (!status[i]) {
                            continue;
                        }
                        // Carry over features which /were/ tracked.
                        features_next[j++] = features_next[i];
                        cv::circle(optflowFrame, features_next[i], 3, cv::Scalar(0, 255, 0), -1, 8);
                        if (features_next[i] != features_prev[i]) {
                            cv::line(optflowFrame, features_prev[i], features_next[i], cv::Scalar(0, 255, 0), 1);
                        }
                    }
                    features_next.resize(j);
                    std::printf("darknet is %d frames behind optflow\n", frameCounter - network.analyzedFrame);
                    this->draw_boxes(optflowFrame, opticalFlowBox);
                }

        }


        // TODO: C++ thrashed me and wouldn't let me check if the past targets were equal.
        // TODO: Hopefully I don't prank myself and someone else in the future fixes this
        // TODO: In all honestly it will probably be me though.
    }

    std::vector<bbox_t> CubeTracker::extrapolate_bbox_through_queue(std::vector<bbox_t> original_bbox, std::queue<cv::Mat> track_queue) {
        if (original_bbox.empty()) {
            return original_bbox;
        }
        std::vector<cv::Point2f> features_prev, features_next;
        auto opticalFlowBox = original_bbox;
        for (auto &i : original_bbox) {
            float centerX = i.x + (i.w / 2.0F);
            float centerY = i.y + (i.h / 2.0F);
            cv::Point2f centerPoint;
            centerPoint.x = centerX;
            centerPoint.y = centerY;
            features_next.push_back(centerPoint);
        }
        features_prev = features_next;

        while (track_queue.size() > 1) {
            cv::Mat current_frame(track_queue.front().size(),
                                  CV_8UC1); // Initialize greyscale current frame mat
            cv::cvtColor(track_queue.front(), current_frame, CV_BGR2GRAY,
                         1); // Convert front of queue to greyscale and put it in current_frame
            track_queue.pop();

            cv::Mat next_frame(track_queue.front().size(), CV_8UC1);
            cv::cvtColor(track_queue.front(), next_frame, CV_BGR2GRAY,
                         1); // Convert front of queue to greyscale and put it in next_frame
            track_queue.pop();

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
                    cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                    cv::OPTFLOW_USE_INITIAL_FLOW
            );

            if (features_next.empty()) {
                std::printf("rip, couldn't track it\n");
                return opticalFlowBox;
            }

            auto optflowFrame = current_frame.clone();

            // update bboxs in opticalFlowBox
            for (size_t i = 0; i < original_bbox.size(); i++) {
                cv::Point2f point_next = features_next.at(i);
                opticalFlowBox.at(i).x = static_cast<unsigned int>(point_next.x - original_bbox.at(i).w / 2);
                opticalFlowBox.at(i).y = static_cast<unsigned int>(point_next.y - original_bbox.at(i).h / 2);
            }
            size_t i, j;
            for (i = j = 0; i < features_next.size(); i++) {
                // Check if the feature was successfully tracked, if not then skip.
                if (!status[i]) {
                    continue;
                }
                // Carry over features which /were/ tracked.
                features_next[j++] = features_next[i];

            }
            features_next.resize(j);
            if (features_next.empty()) {
                return opticalFlowBox;
            }
        }
        return opticalFlowBox;
    }

#pragma clang diagnostic pop

    void CubeTracker::draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec) {
        for (auto &i : result_vec) {
            cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), cv::Scalar(50, 200, 50), 3);
        }
    }

}
