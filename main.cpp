#include "field.hpp"
#include "vision.hpp"
#include <thread>

int main() {
	Field field;
	printf("Shared memory address: %p\n", (void *)&Vision::frame);
	std::thread cameraCapture(Vision::captureImages);
	while (1) {
		field.tick();
	}
}
