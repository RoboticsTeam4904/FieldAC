#include "lidar.hpp"
#include <rplidar.h>
#include <stdio.h>
#include <stdlib.h>

Lidar::Lidar(std::string path, _u32 baudrate = 115200) : path(path), baudrate(baudrate) {
    this->driver = rplidar::RPlidarDriver::CreateDriver(rplidar::RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    if (!this->driver) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
}

bool Lidar::checkHealth() {
    u_result result;
    rplidar_response_device_health_t healthInfo;
    result = this->driver->getHealth(healthInfo);
    if(IS_OK(result)) {
        std::printf("RPLidar Health Status: %d\n", healthInfo.status);
        if(healthInfo.status == RPLIDAR_STATUS_ERROR) {
            std::fprintf(stderr, "Error; rplidar internal error detected. Please reboot the device to retry.\n");
            return false;
        }
        return true;
    } else {
        std::fprintf(stderr, "Error; cannot retrieve the lidar health code: %x\n", result);
        return false;
    }
};

void Lidar::run() {
    auto resp = this->driver->connect(this->path.c_str(), baudrate);
    if(IS_FAIL(resp)) {
        std::fprintf(stderr, "Error; cannot get device info.\n");
        //TODO: Figure out where to handle the errors ohhhh noooo.
    }

    rplidar_response_device_info_t 
};

void initLidar(){
    // make connection...
    if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
        goto on_finished;
    }

    rplidar_response_device_info_t devinfo;

    // retrieving the device info
    ////////////////////////////////////////
    op_result = drv->getDeviceInfo(devinfo);

    if (IS_FAIL(op_result)) {
        fprintf(stderr, "Error, cannot get device info.\n");
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
                   "Firmware Ver: %d.%02d\n"
                   "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        goto on_finished;
    }

    drv->startMotor();
    // start scan...
    drv->startScan();
}
void lidarThread(bool* stop){
    while (1) {
        rplidar_response_measurement_node_t nodes[360*2];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                       (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
                       (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                       nodes[pos].distance_q2/4.0f,
                       nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
        }

        if (*stop){
            break;
        }
    }
}
void exitLidar(){
    drv->stop();
    drv->stopMotor();
    // done!
    on_finished:
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}


LidarScan::LidarScan() = default;
LidarScan::LidarScan(const LidarScan& other, int newOffset) {
    memcpy(distances, other.distances, 360*(sizeof(float)));
    offset = other.offset + newOffset;
    if (offset >= 360) {
        offset -= 360;
    }
}

float LidarScan::compare(const LidarScan& expected) {
    float err = 0;
    for(int i = 0; i < 360; i++) {
        if(getAtAngle(i) < expected.getAtAngle(i)) {
            err += (expected.getAtAngle(i) - getAtAngle(i)) * 0.1;
        } else {
            err += (getAtAngle(i) - expected.getAtAngle(i));
        }
    }
    return err;
}

LidarScan LidarScan::getAtLocation(int xCm, int yCm){
    //TODO: Implement getAtLocation functionality
    return LidarScan();
}
LidarScan LidarScan::generateExpected(const Pose& pose){
    return LidarScan(getAtLocation((int)pose.x,(int)pose.y), (int)(pose.yaw*180/3.14159));
}