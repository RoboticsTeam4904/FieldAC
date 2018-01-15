#include "vision.hpp"

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

    cv::namedWindow("OpenCV Camera", cv::WINDOW_AUTOSIZE);
}

void Vision::captureImages() {
    while (true) {
		frameMutex.lock();
		devCapture.read(frame);
		frameMutex.unlock();
		if(frame.empty()) {
			std::printf(" --(!) No captured frame -- Break!");
			return;
		}
	}
}

bool Vision::displayImage() {
    cv::Mat* frame = getFrame();
    if(frame->empty()) {
//        std::printf(" --(!) No captured frame -- Break!");
        return false;
    }
    cv::imshow("OpenCV Camera", *frame);
    return (cv::waitKey(10) == 27 );
}

cv::Mat* Vision::getFrame() {
	return &frame;
}
