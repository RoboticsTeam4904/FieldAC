#include "field.h"
#include "vision.h"
#include <thread>

int main() {
	Field field;
	printf("Shared memory address: %p\n", (void *)&Vision::frame);
	std::thread cameraCapture(Vision::captureImages);
	while (1) {
		field.tick();
	}
}