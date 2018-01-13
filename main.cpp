#import "field.cpp"
#import "vision.cpp"
#include <opencv2/core.hpp>
#include <stdio.h>
#include <thread>
#include <functional>

using namespace std;

int main() {
	Field f;
	Mat frame;
	printf("Shared memory address: %p\n", (void *)&frame);
	thread cameraCapture(bind(vision::captureImages, &frame));
	while (1) {
		f.tick(frame);
	}
	// vision::displayImage(&frame);
}