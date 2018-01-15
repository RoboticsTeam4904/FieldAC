#include "vision.hpp"
//#include "darknet/arapaho.hpp"
#include "darknet/image.h"
#include <opencv2/highgui.hpp>

cv::Mat Vision::frame;
std::mutex Vision::frameMutex;
cv::VideoCapture Vision::devCapture = cv::VideoCapture();

void Vision::init() {
    devCapture.open(0);
    if (!devCapture.isOpened()) {
        std::printf("--(!) Error opening video capture\n");
        return;
    }
}

void Vision::captureImages() {
    while (true) {
		frameMutex.lock();
		devCapture.read(frame);
		frameMutex.unlock();
		if(frame.empty()) {
			return;
		}
	}
}

bool Vision::displayImage(cv::Mat* frame, const std::string window) {
    cv::namedWindow(window, cv::WINDOW_AUTOSIZE);
    if(frame->empty()) {
        return false;
    }
    cv::imshow(window, *frame);
    return (cv::waitKey(10) == 27 );
}

cv::Mat* Vision::getFrame() {
	return &frame;
}
