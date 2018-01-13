#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

namespace vision {
	Mat frame;
	mutex frameMutex;

    void captureImages() {
    	VideoCapture capture;
	    capture.open( 0 );
	    if (!capture.isOpened()) { printf("--(!)Error opening video capture\n"); return; }
	    while (true) {
	    	frameMutex.lock();
	    	capture.read(frame);
	    	frameMutex.unlock();
	    	if(frame.empty()) {
	            printf(" --(!) No captured frame -- Break!");
	            return;
	        }
	    }
    }
    void displayImage(Mat* frame) {
    	while (1) {
    		if(frame->empty()) {
	            printf(" --(!) No captured frame -- Break!");
	            continue;
	        }
			imshow("Image from camera", *frame);
			if( waitKey(10) == 27 ) { return; }
		}
    }
    Mat* getFrame() {
    	return &vision::frame;
    }
}