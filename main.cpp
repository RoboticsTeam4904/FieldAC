#import "field.cpp"
#import "vision.cpp"
#include <opencv2/core.hpp>
#include <stdio.h>
#include <thread>
#include <functional>

using namespace std;

int main(){
	Field f;
	Mat frame;
	printf("Shared memory address: %p\n", (void *)&frame);
	std::thread cameraCapture(std::bind(vision::captureImages, &frame));
	vision::displayImage(&frame);
	// while(1) {
	// 	f.tick();
	// }
}