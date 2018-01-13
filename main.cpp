#import "field.cpp"
#import "vision.cpp"
#include <opencv2/core.hpp>
#include <stdio.h>
#include <thread>
#include <functional>

using namespace std;

int main() {
	Field f;
	printf("Shared memory address: %p\n", (void *)&vision::frame);
	thread cameraCapture(vision::captureImages);
	while (1) {
		f.tick();
	}
	// vision::displayImage(&frame);
}