#include<stdio.h>
#include<math.h>
#include<string>
#include <iostream>
#include <cstdlib>
#include "../objects.cpp"
#define RAND (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

#define SAMPLES 1000

using namespace std;


	 
void step(Pose input[SAMPLES], float measuredForwardAccel, float measuredYawAccel, string sensorData){
		Pose n[SAMPLES];
		float weights[SAMPLES];
		float weightsSum=0;
		for(int i=0; i<SAMPLES; i++){	
			n[i]=Pose(input[i],measuredForwardAccel,measuredYawAccel);
			weights[i]=n[i].plausibility(sensorData);
			weightsSum+=weights[i];
		}
		for(int i=0; i<SAMPLES; i++){
			float weight=(float)(weightsSum * RAND);
			for(int j=0; j<SAMPLES; j++){
				weight-=weights[j];
				if(weight<0){
					input[i]=n[j];
					break;
				}
			}
		}
	}

// int main(){
// 	Pose data[SAMPLES];
// 	for(int i=0; i<SAMPLES; i++){

// 		data[i].seed();
// 		cout<<data[i].forwardRate<<" ";
// 	}
// 	for(int i=0; i<1000; i++){
// 	step(data,0,0,"");
// }
// cout<<"kys\n\n\nkys\n";
	// for(int i=0; i<SAMPLES; i++){
	// 	cout<<data[i].forwardRate<<" ";
	// }

// }
