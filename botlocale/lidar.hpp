#ifndef PROV_LIDAR_HPP
#define PROV_LIDAR_HPP

#include <cstddef>
#include <rplidar.h>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include "../objects.hpp"
#include <vector>
#include <tuple>

#ifndef _countof
#define _countof(_Array) (int)((sizeof(_Array)) / (sizeof(_Array[0])))
#endif

using namespace rp::standalone;

class Lidar {
protected:
	_u32 baudrate;
	std::string path;
public:
	rplidar::RPlidarDriver *driver;
public:
	Lidar(std::string path, _u32 baudrate);
	void run(const bool* stop);
	void stop();
	bool checkHealth();
};

class LidarScan {
public:
	std::tuple<float, float> measurements[360];
	int offset;
public:
	LidarScan();
	LidarScan(const LidarScan& other, int newOffset);
	float compare(const LidarScan& expected);
	LidarScan generateExpected(const Pose& pose);
	LidarScan getAtLocation(int xCm, int yCm);
	std::vector<float> raytrace(float x, float y);
private:
	inline float getAtAngle(int angle) const {
		int o = angle + offset;
		if(o >= 360)
			o-=360;
		return std::get<0>(measurements[o]);
	}

	float calc(float amount, float x, float y, float t1);
};
#endif
