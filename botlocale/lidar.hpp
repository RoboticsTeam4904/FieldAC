//
// Created by Howard Stark on 1/27/18.
//

#ifndef INC_2018_FIELD_LIDAR_HPP
#define INC_2018_FIELD_LIDAR_HPP

#include "../objects.hpp"

class lidar {

};

class lidarscan{
public:
	float distances[360];
	int offset;
	lidarscan(){

	}
	lidarscan(const lidarscan& other,int newOffset) {
		memcpy(distances, other.distances, 360*(sizeof(float)));
		offset=other.offset+newOffset;
		if (offset >= 360){
			offset-=360;
		}
	}
	~lidarscan(){

	}
private:
	 inline  float getAtAngle(int angle)const{
		int o=angle+offset;
		if(o>=360){
			o-=360;
		}
		return distances[o];
	}
public:
	float compare(const lidarscan& expected){
		float err=0;
		for(int i=0; i<360; i++){
			if(getAtAngle(i) < expected.getAtAngle(i)){
				err += (expected.getAtAngle(i)-getAtAngle(i)) * 0.1;
			}else{
				err += (getAtAngle(i)-expected.getAtAngle(i));
			}
		}
		return err;
	}
};
lidarscan getAtLocation(int xCm, int yCm){
}
lidarscan generateExpected(const Pose& pose){
	return lidarscan(getAtLocation((int)pose.x,(int)pose.y), (int)(pose.yaw*180/3.14159));
}


#endif //INC_2018_FIELD_LIDAR_HPP
