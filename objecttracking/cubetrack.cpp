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
        this->lastFrame = frameUpdate;
        this->track_optflow_queue.push(frameUpdate);
    }

    void CubeTracker::run() {
        std::printf("Running CubeTracker in here!\n");
        cv::Mat frame;
        float cur_time_extrapolate = 0, old_time_extrapolate = 0;
        while(true) {
            std::printf("in the loop\n");
            frame = this->lastFrame;
            std::printf("Frame height: %d\n", frame.size().height);
            if (frame.empty()) {
                std::printf("Frame was empty, continuing\n");
                continue;
            } else {
                std::printf("Frame wasn't empty - %d\n", frame.channels());
            }
            cv::Mat greyscale;
            cv::imshow(" result ", frame);
            std::printf("about to do the fl o w \n");
            int passed_flow_frames = 0;
            std::vector<bbox_t> opticalFlowBox;
            opticalFlowBox = this->targets;
            if (track_optflow_queue.size() > 0) {
                cv::Mat first_frame = track_optflow_queue.front();
                tracker_flow->update_tracking_flow(track_optflow_queue.front(), opticalFlowBox);

                while (track_optflow_queue.size() > 1) {
                    track_optflow_queue.pop();
                    opticalFlowBox = tracker_flow->tracking_flow(track_optflow_queue.front(), true);
                }
                track_optflow_queue.pop();
                passed_flow_frames = 0;

                opticalFlowBox = network.tracking_id(opticalFlowBox);
                auto tmp_result_vec = network.tracking_id(this->targetsLast, false);

                extrapolate_coords.new_result(tmp_result_vec, old_time_extrapolate);
                old_time_extrapolate = cur_time_extrapolate;
                extrapolate_coords.update_result(opticalFlowBox, cur_time_extrapolate - 1);
            }
        }

        // TODO: C++ thrashed me and wouldn't let me check if the past targets were equal.
        // TODO: Hopefully I don't prank myself and someone else in the future fixes this
        // TODO: In all honestly it will probably be me though.
    }
}
