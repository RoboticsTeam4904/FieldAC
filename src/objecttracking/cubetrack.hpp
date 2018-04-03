#ifndef PROV_CUBETRACK_H
#define PROV_CUBETRACK_H

#ifndef OPENCV
#define OPENCV 1
#endif
#ifndef TRACK_OPTFLOW
#define TRACK_OPTFLOW 1
#endif

#include <vector>
#include "../objects.hpp"
#include "../network/network.hpp"
#include "../network/target.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>
#include <opencv2/videoio.hpp>
#include "extrapolate.hpp"
#include "sharedqueue.hpp"

namespace ObjectTracking {

    class CubeTracker {
    public:
        static constexpr double DRIFT_COMPENSATE = 2;
        static constexpr double CUBE_SIZE = 31.3;

        cv::Mat optflowFrame;
        cv::Mat optflowFrameLast;

        bool recalc = false;
        cv::VideoWriter optflowWriter;

        std::vector<bbox_t> optflow_targets;
    private:
        mutable std::mutex listenersMutex;
        std::vector<std::function<void (std::vector<Pose>)>> listeners;

        Network &network;
        std::vector<bbox_t> targetsLast;
        std::vector<bbox_t> targets;
        mutable std::mutex mutexTargets;

        mutable std::mutex optflow_targets_mutex;

        Tracker_optflow *tracker_flow;
        mutable std::mutex track_optflow_mutex;
        std::queue<cv::Mat> track_optflow_queue;
        extrapolate_coords_t extrapolate_coords;

        bool newTargets;
        int frameCounter;
    public:
        explicit CubeTracker(Network &network);
        void update(std::vector<bbox_t>);
        void update(cv::Mat, int);
        void run();
        void draw_boxes(cv::Mat mat_img, std::vector<bbox_t> result_vec, cv::Scalar);
        void registerListener(std::function<void (std::vector<Pose>)> listener);
        std::vector<bbox_t> extrapolate_bbox_through_queue(std::vector<bbox_t>, std::queue<cv::Mat>);
        std::vector<Pose> get_objects();
    private:
        void notifyListeners(std::vector<Pose> objects);
    };

}

#endif
