#include "vision.hpp"

#include <opencv2/highgui.hpp>

cv::Mat Vision::frame;
std::mutex Vision::frameMutex;

void Vision::captureImages() {
	cv::VideoCapture capture;
	capture.open(0);
	if (!capture.isOpened()) {
		std::printf("--(!)Error opening video capture\n");
		return;
	}
	while (true) {
		frameMutex.lock();
		capture.read(frame);
		frameMutex.unlock();
		if(frame.empty()) {
			std::printf(" --(!) No captured frame -- Break!");
			return;
		}
	}
}

void Vision::displayImage(cv::Mat* frame) {
	while (true) {
		if(frame->empty()) {
			std::printf(" --(!) No captured frame -- Break!");
			continue;
		}
		cv::imshow("Image from camera", *frame);
		if (cv::waitKey(10) == 27 ) {
			return;
		}
	}
}

cv::Mat* Vision::getFrame() {
	return &frame;
}
