//
// Created by Howard Stark on 1/27/18.
//

#ifndef PROV_LIDAR_HPP
#define PROV_LIDAR_HPP

#include <rplidar.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include "../objects.hpp"

#ifndef _countof
#define _countof(_Array) (int)((sizeof(_Array)) / (sizeof(_Array[0])))
#endif

using namespace rp::standalone;

class Lidar {
protected:
	_u32 baudrate;
	std::string path;
	u_result result;
public:
	rplidar::RPlidarDriver *driver;
public:
	Lidar(std::string path, _u32 baudrate = 115200);
	void run();
	bool checkHealth();
};

class LidarScan {
protected:
	float distances[360];
	int offset;
public:
	LidarScan();
	LidarScan(const LidarScan& other, int newOffset);
	float compare(const LidarScan& expected);
	LidarScan generateExpected(const Pose& pose);
	LidarScan getAtLocation(int xCm, int yCm);
private:
	inline float getAtAngle(int angle) const {
		int o = angle + offset;
		if(o >= 360)
			o-=360;
		return distances[o];
	}
};
#endif
