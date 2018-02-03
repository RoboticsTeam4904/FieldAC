//
// Created by Howard Stark on 1/27/18.
//

#ifndef INC_2018_FIELD_LIDAR_HPP
#define INC_2018_FIELD_LIDAR_HPP


class lidar {

};


class lidarscan{
	float distances[360];
	lidarscan(){

	}
	~lidarscan(){

	}
	float compare(const lidarscan& expected){
		float err=0;
		for(int i=0; i<360; i++){
			if(distances[i] < expected.distances[i]){
				err += (expected.distances[i]-distances[i]) * 0.1;
			}else{
				err += (distances[i]-expected.distances[i]);
			}
		}
		return err;
	}
}


#endif //INC_2018_FIELD_LIDAR_HPP
