#ifndef PROV_CUBETRACK_H
#define PROV_CUBETRACK_H

#ifndef OPENCV
#define OPENCV 1
#endif
#ifndef TRACK_OPTFLOW
#define TRACK_OPTFLOW 1
#endif
#ifdef GPU
#undef GPU
#endif

#include <vector>
#include "../objects.hpp"
#include "../network/network.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/videoio.hpp>
#include "../network/target.hpp"
#include "extrapolate.hpp"
#include "sharedqueue.hpp"

namespace ObjectTracking {

    class CubeTracker {
    public:
        cv::Mat optflowFrame;
        cv::Mat optflowFrameLast;

        bool recalc = false;
    private:
        Network &network;
        std::vector<bbox_t> targetsLast;
        std::vector<bbox_t> targets;
        mutable std::mutex mutexTargets;

        Tracker_optflow *tracker_flow;
        std::queue<cv::Mat> track_optflow_queue;
        extrapolate_coords_t extrapolate_coords;

        mutable std::mutex mutexFrame;
        cv::Mat lastFrame;
        cv::Mat lastLastFrame;

        bool newTargets;
        int frameCounter;
    public:
        CubeTracker(Network &network);

        void update(std::vector<bbox_t>);

        void update(cv::Mat, int);

        void run();

        std::vector<bbox_t> extrapolate_bbox_through_queue(std::vector<bbox_t>, std::queue<cv::Mat>);

        void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, const cv::Scalar);
        const double drift_compensate = 2;
    };

}

#endif
