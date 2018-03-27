#include <cmath>
#include "lidar.hpp"
#include "../field.hpp"
#include <tuple>

#define LIDAR_OFFSET = -3*PI / 4

Lidar::Lidar(std::string path, _u32 baudrate) : path(path), baudrate(baudrate) {
    this->driver = rplidar::RPlidarDriver::CreateDriver(rplidar::RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    if (!this->driver) {
        std::fprintf(stderr, "Insufficient memory, exit\n");
        exit(-2);
    }
}

bool Lidar::checkHealth() {
    rplidar_response_device_health_t healthInfo;
    auto healthResp = this->driver->getHealth(healthInfo);
    if (IS_OK(healthResp)) {
        std::printf("RPLidar Health Status: %d\n", healthInfo.status);
        if (healthInfo.status == RPLIDAR_STATUS_ERROR) {
            std::fprintf(stderr, "Error; rplidar internal error detected. Please reboot the device to retry.\n");
            return false;
        }
        return true;
    } else {
        std::fprintf(stderr, "Error; cannot retrieve the lidar health code: %x\n", healthResp);
        return false;
    }
};

void Lidar::run(const bool *stop) {
    
    auto connResp = this->driver->connect(this->path.c_str(), baudrate);
    if (IS_FAIL(connResp)) {
        std::fprintf(stderr, "Error; cannot bind to the specified serial port %s.\n", this->path.c_str());
        this->stop();
    }

    rplidar_response_device_info_t devInfo;
    auto deviResp = this->driver->getDeviceInfo(devInfo);
    if (IS_FAIL(deviResp)) {
        std::fprintf(stderr, "Error; cannot get device info.\n");
        this->stop();
    }

    std::printf("RPLIDAR S/N: ");
    for (unsigned char pos : devInfo.serialnum) {
        std::printf("%02X", pos);
    }
    std::printf("\n");

    std::printf("Firmware Ver: %d.%02d\n"
                        "Hardware Rev: %d\n",
                devInfo.firmware_version >> 8,
                devInfo.firmware_version & 0xFF,
                (int) devInfo.hardware_version);

    if (!this->checkHealth()) {
        this->stop();
    }

    this->driver->startMotor();
    this->driver->startScan();

    while (true) {
        //TODO: This should be rewritten to respect C++11 idioms.
        //TODO: Unfortunately, that might involve a deeper wrapper for proper destructors hidden away...
        rplidar_response_measurement_node_t nodes[360 * 2];
        size_t count = _countof(nodes);
        auto scanResp = this->driver->grabScanData(nodes, count);
        if (IS_OK(scanResp)) {
            this->driver->ascendScanData(nodes, count);
            LidarScan tmp;

            for (int pos = 0; pos < (int) count; pos++) {
                float angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
                tmp.measurements[((int) (angle + 0.5f)) % 360] = std::make_tuple(angle, nodes[pos].distance_q2 / 4.0f);
                std::get<0>(tmp.measurements[((int) (angle + 0.5f)) % 360]) /= 10; // convert to centimeters
//                std::printf("%s theta: %03.2f Dist: %08.2f Q: %d\n",
//                            (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
//                            (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
//                            nodes[pos].distance_q2 / 4.0f,
//                            nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
            tmp.offset = LIDAR_OFFSET;
            this->current_scan = tmp;
        }

        if (*stop) {
            break;
        }
    }
};

void Lidar::stop() {
    this->driver->stop();
    this->driver->stopMotor();

    rplidar::RPlidarDriver::DisposeDriver(this->driver);
}

LidarScan::LidarScan() = default;

LidarScan::LidarScan(const LidarScan &other, int newOffset) {
    std::copy(&other.measurements[0], &other.measurements[360], measurements);
    offset = other.offset + newOffset;
    if (offset >= 360) {
        offset -= 360;
    }
    offset %= 10 * 3 * 2 * 6;
    while (offset >= 360) {
        offset -= 360;
    }
}

LidarScan LidarScan::getAtLocation(int xCm, int yCm) {
    //TODO: Implement getAtLocation functionality
    //call raytrace and convert result to lidarscan
    return LidarScan();
}

LidarScan LidarScan::generateExpected(const Pose &pose) {
    return LidarScan(getAtLocation((int) pose.x, (int) pose.y), (int) (pose.yaw * 180 / M_PI));
}

cv::Point2f *LidarScan::intersect_ray_with_segment(cv::Point2f origin, cv::Vec2f direction, Segment seg) {
    auto start = tuple_to_point(seg.start);
    auto end = tuple_to_point(seg.end);
    cv::Point2f *intersection = static_cast<cv::Point2f *>(malloc(1 * sizeof(cv::Point2f)));

    auto v1 = origin - start;
    auto v2 = end - start;
    auto v3 = cv::Vec2f(-direction[1], direction[0]);


    auto dot = v2.dot(v3);
    if (std::abs(dot) < 0.000001)
        return nullptr;

    auto t1 = v2.cross(v1) / dot;
    auto t2 = (v1.dot(v3)) / dot;

    if (t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0)) {
        *intersection = origin + cv::Point2f((t1 * direction));
        return intersection;
    }

    return nullptr;
}

struct SegmentComparator {
    explicit SegmentComparator(std::tuple<double, double> robot_pos) { this->pos = robot_pos; }

    bool operator()(Segment i, Segment j) {
        cv::Point2f i_midpoint = cv::Point2f(std::get<0>(i.start), std::get<1>(i.start)) +
                                 ((cv::Point2f(std::get<0>(i.start), std::get<1>(i.start)) -
                                   cv::Point2f(std::get<0>(i.end), std::get<1>(i.end))) / 2);
        cv::Point2f j_midpoint = cv::Point2f(std::get<0>(j.start), std::get<1>(j.start)) +
                                 ((cv::Point2f(std::get<0>(j.start), std::get<1>(j.start)) -
                                   cv::Point2f(std::get<0>(j.end), std::get<1>(j.end))) / 2);
        auto i_dist = pow(pow(std::get<0>(pos) - i_midpoint.x, 2) + pow(std::get<1>(pos) - i_midpoint.y, 2), 0.5);
        auto j_dist = pow(pow(std::get<0>(pos) - j_midpoint.x, 2) + pow(std::get<1>(pos) - j_midpoint.y, 2), 0.5);
        return (i_dist < j_dist);
    }

    std::tuple<double, double> pos;
};

double LidarScan::raytrace(Pose robot_pose) {
    double robot_x = robot_pose.x;
    double robot_y =  robot_pose.y;
    float yaw_degrees = robot_pose.yaw;
    std::tuple<double, double> pos = std::tuple<double, double>(robot_x, robot_y);
    double err = 0;

    for (auto measurement : measurements) {
        auto rads = (std::get<0>(measurement) - yaw_degrees) * M_PI / 180;
        auto direction = cv::Vec2f(std::cos(rads), std::sin(rads));

        std::vector<Segment> sorted_segments = Field::getInstance()->construct;
        std::sort(sorted_segments.begin(), sorted_segments.end(), SegmentComparator(pos));
        for (auto seg : sorted_segments) {
            auto intersection = this->intersect_ray_with_segment(tuple_to_point(pos), direction, seg);
            if (intersection) {
                cv::Point2f expected_point = *intersection;
                cv::Point2f actual_point = cv::Point2f(direction * std::get<1>(measurement));
                // angle should be the same because math so we just compare magnitudes
                auto expected_mag = cv::norm(expected_point);
                auto actual_mag = cv::norm(actual_point);
                actual_mag = std::get<1>(measurement); // those SHOULD be the same
//                std::printf("%f should be the same as %f\n", cv::norm(actual_point), actual_mag);
                if (actual_mag < expected_mag) {
                    err += (expected_mag - actual_mag) * 0.1;
                } else {
                    err += (actual_mag - expected_mag);
                }
                free(intersection);
                break;
            }
        }
    }
    return err;
}

float LidarScan::calc(float amount, float x, float y, float t1) {
    auto seg = Field::getInstance()->construct[0];
    auto angle = std::get<0>(this->measurements[0]);
//    std::tan(angle * (180 / 3.14159)) * (t1 - x);
    return 0.0f;
}

cv::Point2f tuple_to_point(std::tuple<double, double> t) {
    return cv::Point2f(std::get<0>(t), std::get<1>(t));
}

//
//float[] raytrace(float x, float y){
//    float dists[360];
//    outer:
//    for(int i=0; i<360; i++){
//        for(int seg=0; seg<field.length; seg++){
//            float[][] s=field[seg];
//            float* intersect=intersectRayWithLineSegment(x,y,i,s[0][0],s[0][1],s[1][0],s[1][1]);
//            if(intersect){
//                dists[i]=sqrt((x-intersect[0])**2 + (y-intersect[1])**2)
//                free(intersect) idk lol
//                continue outer
//            }
//        }
//        dists[i]=-1
//    }
//}
//
//float* intersectRayWithLineSegment(float centerX, float centerY, int rayAngleDegrees, float lineSegmentStartX, float lineSegmentStartY, float lineSegmentEndX, float lineSegmentEndY){
//    returns 0 if no intersection, otherwise returns pointer to intersection x,y
//    get this from stackoverflow
//}