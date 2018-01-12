#include<stdio.h>
#include<math.h>
#include<string>
#include <iostream>
#include <cstdlib>
#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

#define SAMPLES 1000

using namespace std;


struct Pose{
	Pose(){

	}
	 Pose(Pose prev, float measuredForwardAccel, float measuredYawAccel){
		x=prev.x + cos(prev.yaw) * prev.forwardRate + RAND*0.001-0.0005;
		y=prev.y + sin(prev.yaw) * prev.forwardRate + RAND*0.001-0.0005;

		forwardRate = prev.forwardRate + measuredForwardAccel + RAND*0.001-0.0005;

		yaw = prev.yaw + prev.yawRate ;
		yawRate = prev.yawRate + measuredYawAccel ;
	}
	void seed(){
		x=RAND;
		y=RAND;
		yaw=0;
		forwardRate=RAND;
		yawRate=0;
	}
	float x;
	float y;
	float yaw;
	float forwardRate;
	float yawRate;
	float plausibility(string sensorData){
		if(x<-10 || x>10 || y<-10 || y>10){
			//cout<<"aoeu\n";
			return 0.1;
		}
		return 1;
	}
};