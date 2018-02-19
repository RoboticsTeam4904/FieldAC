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
        std::printf("large nut");
        this->mutexFrame.lock();
        this->lastFrame = frameUpdate;
        this->mutexFrame.unlock();
        this->track_optflow_queue.push(frameUpdate);
    }

    void CubeTracker::run() {
        std::printf("Running CubeTracker in here!\n");
        cv::Mat frame;
        float cur_time_extrapolate = 0, old_time_extrapolate = 0;
        while(true) {
            this->mutexFrame.lock();
            frame = this->lastFrame.clone();
            this->mutexFrame.unlock();
//            std::printf("Frame height: %d\n", frame.size().height);
            if (frame.empty()) {
                continue;
            }
            std::vector<bbox_t> opticalFlowBox;
            opticalFlowBox = this->targets;
            if (!opticalFlowBox.size()) {
                continue;
            }

            auto old_result_vec = network.tracking_id(opticalFlowBox);
            if (track_optflow_queue.size() > 0) {
                cv::Mat first_frame = track_optflow_queue.front();
                tracker_flow->update_tracking_flow(track_optflow_queue.front(), opticalFlowBox);

                while (track_optflow_queue.size() > 1) {
                    track_optflow_queue.pop();
                    opticalFlowBox = tracker_flow->tracking_flow(track_optflow_queue.front(), true);
                }
                track_optflow_queue.pop();

                opticalFlowBox = network.tracking_id(opticalFlowBox);
                for (int i = 0; i < opticalFlowBox.size(); ++i) {
                    std::printf("%d, %d", opticalFlowBox[i].x, opticalFlowBox[i].y);
                }

                auto tmp_result_vec = network.tracking_id(this->targetsLast, false);

                extrapolate_coords.new_result(tmp_result_vec, old_time_extrapolate);
                old_time_extrapolate = cur_time_extrapolate;
                extrapolate_coords.update_result(opticalFlowBox, cur_time_extrapolate - 1);
            }
            // add old tracked objects
            for (auto &i : old_result_vec) {
                auto it = std::find_if(opticalFlowBox.begin(), opticalFlowBox.end(),
                                       [&i](bbox_t const& b) { return b.track_id == i.track_id && b.obj_id == i.obj_id; });
                bool track_id_absent = (it == opticalFlowBox.end());
                if (track_id_absent) {
                    if (i.frames_counter-- > 1)
                        opticalFlowBox.push_back(i);
                }
                else {
                    it->frames_counter = std::min((unsigned)3, i.frames_counter + 1);
                }
            }
            this->tracker_flow->update_cur_bbox_vec(opticalFlowBox);
            opticalFlowBox = this->tracker_flow->tracking_flow(frame, true);
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
