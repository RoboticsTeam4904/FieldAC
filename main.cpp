#include "field.hpp"
#include "vision.hpp"
#include <thread>
#include <opencv/cv.hpp>

int main() {
    Field field;

    Vision::init();
    printf("Shared memory address: %p\n", (void *) &Vision::frame);
    std::thread cameraCapture(Vision::captureImages);

    while (1) {
        field.tick();
        if (Vision::displayImage(Vision::getFrame(), "OpenCV Camera")) {
            return 1;
        }
    }
}
