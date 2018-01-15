#include "mcl.hpp"

#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

void BotLocale::step(Pose input[SAMPLES], const float measuredAccelForward, const float measuredAccelYaw, std::string sensorData){
	Pose n[SAMPLES];
	float weights[SAMPLES];
	float weightsSum=0;
	for(int i=0; i<SAMPLES; i++){
		n[i]=Pose(input[i],measuredAccelForward,measuredAccelYaw);
		weights[i]=n[i].plausibility(sensorData);
		weightsSum+=weights[i];
	}
	for(int i=0; i<SAMPLES; i++){
		float weight= weightsSum * RAND;
		for(int j=0; j<SAMPLES; j++){
			weight-=weights[j];
			if(weight<0){
				input[i]=n[j];
				break;
			}
		}
	}
}

void BotLocale::tick() {

}